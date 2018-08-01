
/// PROJECT
#include <csapex/model/node.h>

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

class DegToRad : public csapex::Node {
public:

    DegToRad() :
        angle_(0),
        publish_(false)
    {
    }

    virtual void setup(NodeModifier &node_modifier) override
    {
        output_ = node_modifier.addOutput<double>("rad");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::factory::declareRange("angle", -180.0, 180.0, 0.0, 0.01),
                                std::bind(&DegToRad::publish, this));
    }

    virtual void process() override
    {
        if(publish_) {
            msg::publish(output_, degToRad(angle_));
            publish_ = false;
        }
    }


private:
    Output *output_;
    double  angle_;
    bool    publish_;

    void publish()
    {
        angle_ = readParameter<double>("angle");
        publish_ = true;
    }

};
}

CSAPEX_REGISTER_CLASS(csapex::DegToRad, csapex::Node)


