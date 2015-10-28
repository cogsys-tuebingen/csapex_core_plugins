
/// PROJECT
#include <csapex/model/tickable_node.h>

#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>

#include <csapex/msg/generic_value_message.hpp>

#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

namespace csapex {
using namespace connection_types;

inline double degToRad(const double deg)
{
    const static double fac = M_PI / 180.0;
    return fac * deg;
}

class DegToRad : public csapex::TickableNode {
public:

    DegToRad() :
        last_angle_(0.0)
    {
    }

    virtual void setup(NodeModifier &node_modifier) override
    {
        output_ = node_modifier.addOutput<double>("rad");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("angle", -180.0, 180.0, 0.0, 0.01));
    }

    virtual void tick() override
    {
        msg::publish(output_, degToRad(last_angle_));
    }

    virtual bool canTick() override
    {
        double angle = readParameter<double>("angle");
        if(angle != last_angle_) {
            last_angle_ = angle;
            return true;
        }
        return false;
    }

    virtual void process() override
    {
    }


private:
    Output *output_;
    double last_angle_;
};
}

CSAPEX_REGISTER_CLASS(csapex::DegToRad, csapex::Node)


