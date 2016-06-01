
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>

#include <csapex_scan_2d/scan_message.h>
#include <csapex/msg/generic_value_message.hpp>

#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

#include <utils_laser_processing/common/yaml-io.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>


namespace {
double normalize(double angle)
{
    while (angle <= -M_PI)
        angle += 2 * M_PI;
    while (angle > M_PI)
        angle -= 2 * M_PI;
    return angle;
}
}

namespace csapex {
using namespace lib_laser_processing;
using namespace boost::accumulators;
using namespace connection_types;

class HeightPitchAngle : public csapex::Node {
public:

    HeightPitchAngle() :
        toggle_counter_(0),
        toggle_(false)
    {
    }

    virtual void setup(NodeModifier &node_modifier) override
    {
        input_         = node_modifier.addInput<ScanMessage>("filtered scan");
        output_pitch_  = node_modifier.addOutput<double>("pitch");
        output_height_ = node_modifier.addOutput<double>("height");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("start height", 0.05, 1.0, 0.1, 0.001),
                                std::bind(&HeightPitchAngle::updateStartParameters, this));
        parameters.addParameter(param::ParameterFactory::declareRange("start pitch", -M_PI, M_PI, 0.0, 0.01),
                                std::bind(&HeightPitchAngle::updateStartParameters, this));
        parameters.addParameter(param::ParameterFactory::declareRange("switch after", 1, 100, 1, 1));
        parameters.addParameter(param::ParameterFactory::declareBool("renew", false));

        parameters.addParameter(param::ParameterFactory::declareTrigger("reset"),
                                std::bind(&HeightPitchAngle::reset, this));
        parameters.addParameter(param::ParameterFactory::declareBool("degrees", false));
    }

    virtual void process() override
    {
        ScanMessage::ConstPtr in = msg::getMessage<ScanMessage>(input_);
        const Scan &scan = in->value;
        bool   deg          = readParameter<bool>("degrees");
        int    toggle_after = readParameter<int>("switch after");
        bool   renew        = readParameter<bool>("renew");

        for(const LaserBeam &b : scan.rays) {
            if(b.range() > 0.f)
                mean_dist_(b.posX());
        }

        if(toggle_) {
            /// fit height
            height_ = sin(pitch_) * mean(mean_dist_);
        } else {
            /// fit angle
            pitch_ = normalize(asin(height_ / mean(mean_dist_)));
        }
        ++toggle_counter_;

        if(toggle_counter_ >= toggle_after) {
            toggle_counter_ = 0;
            toggle_ = !toggle_;
            if(renew)
                reset();
        }


        double out_pitch = pitch_;
        if(deg)
            out_pitch *= 180.0 / M_PI;

        msg::publish(output_pitch_, out_pitch);
        msg::publish(output_height_, height_);
    }


private:
    Input  *input_;
    Output *output_pitch_;
    Output *output_height_;
    double  height_;
    double  pitch_;
    int     toggle_counter_;
    bool    toggle_;

    accumulator_set<double, stats<tag::mean>> mean_dist_;

    void reset()
    {
        mean_dist_ = accumulator_set<double, stats<tag::mean>>();
    }

    void updateStartParameters()
    {
        height_ = readParameter<double>("start height");
        pitch_  = readParameter<double>("start pitch");
        reset();
    }

};
}

CSAPEX_REGISTER_CLASS(csapex::HeightPitchAngle, csapex::Node)

