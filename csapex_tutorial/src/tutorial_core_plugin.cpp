/// PROJECT
#include <csapex/core/core_plugin.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/profiling/timer.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_tutorial/TutorialMessage.h>
#include <csapex_tutorial/tutorial_message.h>

namespace csapex
{

namespace tutorial
{

struct ConvertTutorialMessage
{
    static typename connection_types::TutorialMessage::Ptr ros2apex(const typename csapex_tutorial::TutorialMessage::ConstPtr &ros_msg) {
        typename connection_types::TutorialMessage::Ptr out(new connection_types::TutorialMessage);
        out->value = ros_msg->value;
        return out;
    }
    static typename csapex_tutorial::TutorialMessage::Ptr apex2ros(const typename connection_types::TutorialMessage::ConstPtr& apex_msg) {
        typename csapex_tutorial::TutorialMessage::Ptr out(new csapex_tutorial::TutorialMessage);
        out->value = apex_msg->value;
        return out;
    }
};

class RegisterPlugin : public CorePlugin
{
public:
    RegisterPlugin()
        : timer("Application Duration")
    {}

    virtual void prepare(Settings&) override
    {
        timer.restart();

        RosMessageConversion::registerConversion<csapex_tutorial::TutorialMessage, connection_types::TutorialMessage, ConvertTutorialMessage>();
    }

    virtual void shutdown() override
    {
        timer.finish();

        long duration_ms = timer.stopTimeMs() - timer.startTimeMs();

        std::cerr << "the program ran for " << (duration_ms / 1000.) << " seconds" << std::endl;
    }

private:
    csapex::Timer timer;
};

} // namespace tutorial

} // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::tutorial::RegisterPlugin, csapex::CorePlugin)
