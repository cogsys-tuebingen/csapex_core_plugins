/// PROJECT
#include <csapex/csapex_fwd.h>
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_tutorial/tutorial_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

namespace csapex
{
namespace tutorial
{
class UserSwitch : public Node
{
private:
    enum class Selector
    {
        FIRST,
        SECOND
    };

public:
    UserSwitch()
    {
    }

    /// initialize the node
    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        // Connector for the first input image
        in_a_ = node_modifier.addInput<connection_types::CvMatMessage>("Image 1");

        // Connector for the second input image
        in_b_ = node_modifier.addInput<connection_types::CvMatMessage>("Image 2");

        // Connector for the output input image (sub id = 0)
        out_ = node_modifier.addOutput<connection_types::CvMatMessage>("Either image 1 or image 2");

        // Connector that demonstrates TutorialMessage
        out_tutorial_msg_ = node_modifier.addOutput<connection_types::TutorialMessage>("Tutorial Message (true iff FIRST)");
    }

    /// register parameters
    void setupParameters(Parameterizable& parameters) override
    {
        /// for demonstration we map an enum to a set parameter of type int
        std::map<std::string, int> selector = {
            { "FIRST", (int)Selector::FIRST },
            { "SECOND", (int)Selector::SECOND },
        };

        // we register the parameter with the name "selector", the default value of
        // "Selector::FIRST" and a callback lambda function
        parameters.addParameter(param::factory::declareParameterSet("selector", selector, (int)Selector::FIRST), [this](param::Parameter* p) {
            /// this callback is called whenever the parameter
            /// value changes it is also called once before
            /// the first call of process
            selector_ = static_cast<Selector>(p->as<int>());
        });
    }

    /// callback when all messages arrived
    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters) override
    {
        connection_types::CvMatMessage::ConstPtr output_msg;
        // selector_ is set whenever the parameter changes
        switch (selector_) {
            case Selector::FIRST:
                output_msg = msg::getMessage<connection_types::CvMatMessage>(in_a_);
                break;
            case Selector::SECOND:
                output_msg = msg::getMessage<connection_types::CvMatMessage>(in_b_);
                break;
            default:
                throw std::runtime_error("unknown selector");
        }

        msg::publish(out_, output_msg);

        // publish the state of selector_ using a TutorialMessage
        connection_types::TutorialMessage::Ptr tutorial_msg = std::make_shared<connection_types::TutorialMessage>();
        tutorial_msg->value = selector_ == Selector::FIRST;
        msg::publish(out_tutorial_msg_, tutorial_msg);
    }

private:
    Input* in_a_;
    Input* in_b_;

    Output* out_;
    Output* out_tutorial_msg_;

    Selector selector_;
};

}  // namespace tutorial

}  // namespace csapex

// Register the class
CSAPEX_REGISTER_CLASS(csapex::tutorial::UserSwitch, csapex::Node)
