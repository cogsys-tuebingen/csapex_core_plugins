/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

namespace csapex {

using namespace connection_types;

class Magnitude : public csapex::Node
{
public:
    Magnitude()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<CvMatMessage>("Matrix");
        output_ = node_modifier.addOutput<CvMatMessage>("Magnitude");
    }
    virtual void setupParameters(Parameterizable &parameters) override
    {
    }
    virtual void process() override
    {
        CvMatMessage::ConstPtr input = msg::getMessage<CvMatMessage>(input_);
        CvMatMessage::Ptr output(new CvMatMessage(enc::unknown, input->stamp_micro_seconds));

        cv::Mat dx;
        cv::Mat dy;
        cv::Scharr(input->value, dx, CV_32F, 1, 0);
        cv::Scharr(input->value, dy, CV_32F, 0, 1);
        cv::magnitude(dx, dy, output->value);

        msg::publish(output_, output);
    }

private:
    Input*  input_;
    Output* output_;
};
}
CSAPEX_REGISTER_CLASS(csapex::Magnitude, csapex::Node)
