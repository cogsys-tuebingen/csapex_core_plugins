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


namespace csapex {
using namespace lib_laser_processing;
using namespace boost::accumulators;
using namespace connection_types;

class AngleToPlane : public csapex::Node {
public:

    AngleToPlane()
    {

    }

    virtual void setup(NodeModifier &node_modifier) override
    {
        input_  = node_modifier.addInput<ScanMessage>("filtered scan");
        output_ = node_modifier.addOutput<double>("angle");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("height", 0.05, 1.0, 0.1, 0.001));
        parameters.addParameter(param::ParameterFactory::declareTrigger("reset"),
                                std::bind(&AngleToPlane::reset, this));
        parameters.addParameter(param::ParameterFactory::declareBool("degrees", false));
    }

    virtual void process() override
    {
        ScanMessage::ConstPtr in = msg::getMessage<ScanMessage>(input_);
        const Scan &scan = in->value;
        double out = 0.0;
        double height = readParameter<double>("height");
        bool   deg    = readParameter<bool>("degrees");

        for(const LaserBeam &b : scan.rays) {
            if(b.range > 0.f)
                mean_dist_(b.pos_x);
        }

        out = asin(height / mean(mean_dist_));
        if(deg)
            out *= 180.0 / M_PI;

        msg::publish(output_, out);

    }


private:
    Input  *input_;
    Output *output_;


    accumulator_set<double, stats<tag::mean>> mean_dist_;

    void reset()
    {
        mean_dist_ = accumulator_set<double, stats<tag::mean>>();
    }

};
}

CSAPEX_REGISTER_CLASS(csapex::AngleToPlane, csapex::Node)

