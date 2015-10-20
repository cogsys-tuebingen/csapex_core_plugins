#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>

#include <csapex_scan_2d/scan_message.h>
#include <csapex/msg/generic_value_message.hpp>

#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

#include <utils_laser_processing/common/yaml-io.hpp>

#include <atomic>
#include <thread>
#include <ceres/ceres.h>

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
using namespace connection_types;

struct RPResidual {
    RPResidual(double _x, double _y, double _h) :
        x(_x),
        y(_y),
        h(_h)
    {
    }

    template<typename T>
    bool operator()(const T * const alpha, const T * const beta, T * residual ) const {
        residual[0] = T(x) - T(h / (sin(alpha[0]) * cos(beta[0])) - y * (tan(beta[0]) / sin(alpha[0])));
        return true;
    }

    const double x;
    const double y;
    const double h;
};

class RollPitchAngles : public csapex::Node {
public:

    RollPitchAngles() :
        results_(false)
    {
        reset();
    }

    virtual void setup(NodeModifier &node_modifier) override
    {
        input_  = node_modifier.addInput<ScanMessage>("filtered scan");
        pitch_output_ = node_modifier.addOutput<double>("pitch");
        roll_output_  = node_modifier.addOutput<double>("roll");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("height", 0.05, 1.0, 0.1, 0.001));

        parameters.addParameter(param::ParameterFactory::declareTrigger("compute"),
                                std::bind(&RollPitchAngles::compute, this));
        parameters.addParameter(param::ParameterFactory::declareTrigger("reset"),
                                std::bind(&RollPitchAngles::reset, this));

        parameters.addParameter(param::ParameterFactory::declareBool("degrees", false));
    }

    virtual void process() override
    {
        ScanMessage::ConstPtr in = msg::getMessage<ScanMessage>(input_);
        const Scan &scan = in->value;
        double height = readParameter<double>("height");
        bool   deg    = readParameter<bool>("degrees");

        if(height_ == 0.0)
            height_ = height;

        if(collect_.load()) {
            for(const LaserBeam &b : scan.rays) {
                if(b.range > 0.f) {
                    ceres::CostFunction *cost_function =
                            new ceres::AutoDiffCostFunction<RPResidual, 1, 1, 1>(new RPResidual(b.pos_x, b.pos_y, height_));
                    problem_->AddResidualBlock(cost_function, NULL, &pitch_, &roll_);
                }
            }
        }

        if(results_.load()) {
            double pitch = normalize(pitch_);
            double roll = normalize(roll_);
            if(deg) {
                pitch *= 180.0 / M_PI;
                roll *= 180.0 / M_PI;
            }
            msg::publish(pitch_output_, pitch);
            msg::publish(roll_output_, roll);
        }
    }


private:
    Input  *input_;
    Output *pitch_output_;
    Output *roll_output_;

    double           pitch_;
    double           roll_;
    double           height_;
    std::atomic_bool results_;
    std::atomic_bool collect_;

    std::shared_ptr<ceres::Problem> problem_;

    void reset()
    {
        problem_.reset(new ceres::Problem());
        pitch_ = 1.0;
        roll_ = 1.0;
        height_ = 0.0;
        results_.store(false);
        collect_.store(true);
    }

    void compute()
    {
        if(collect_.load()) {
            collect_.store(false);
            std::thread(std::bind(&RollPitchAngles::doCompute, this)).detach();
        }
    }

    void doCompute() {
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        ceres::Solve(options, problem_.get(), &summary);
        results_.store(true);
    }


};
}

CSAPEX_REGISTER_CLASS(csapex::RollPitchAngles, csapex::Node)

