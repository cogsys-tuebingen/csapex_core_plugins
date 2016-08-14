/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

namespace vision_plugins {
using namespace csapex;
using namespace connection_types;

class ContrastBrightness : public csapex::Node
{
public:
    ContrastBrightness()
    {

    }

    virtual void process() override
    {
        CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
        CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));
        out->value = in->value.clone();
        cv::Scalar beta(beta_, beta_, beta_);
        out->value = out->value * alpha_ + beta;
        msg::publish(output_, out);
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<CvMatMessage>("original");
        output_ = node_modifier.addOutput<CvMatMessage>("adjusted");
    }

    virtual void setupParameters(Parameterizable &parameters)
    {
        parameters.addParameter(param::ParameterFactory::declareRange("contrast", 0.1, 4.0, 1.0, 0.01),
                                alpha_);
        parameters.addParameter(param::ParameterFactory::declareRange("brightness", -100, 100, 0, 1),
                                beta_);
    }

protected:
    csapex::Output*   output_;
    csapex::Input*    input_;
    double alpha_;
    int    beta_;
};
}

CSAPEX_REGISTER_CLASS(vision_plugins::ContrastBrightness, csapex::Node)
