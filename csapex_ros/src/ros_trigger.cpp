/// COMPONENT
#include <csapex_ros/ros_node.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex_ros/ros_handler.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/stream_interceptor.h>
#include <csapex/msg/message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/signal/event.h>

namespace csapex {

class RosTrigger : public RosNode
{
public:
    RosTrigger()
    {

    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        trigger_ = node_modifier.addEvent("signalled");
    }
    virtual void setupROS() override
    {

    }
    virtual void processROS() override
    {

    }
    virtual void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(csapex::param::ParameterFactory::declareText("topic", "trigger"),
                                [this](param::Parameter* p) {
            setTopic(p->as<std::string>());
        });
    }

protected:
    void setTopic(const std::string& topic)
    {
        if(topic != current_topic_ && !topic.empty()) {
            current_topic_ = topic;

            ros::master::TopicInfo ti;
            ti.name = current_topic_;
            ti.datatype = "std_msgs::String";
            ainfo << "subscribing to " << ti.name << std::endl;
            sub = RosMessageConversion::instance().subscribe(ti, 1, [this](const TokenDataConstPtr& msg){
                ainfo << "trigger" << std::endl;
                trigger_->trigger();
            });
        }
    }

private:
    Event* trigger_;
    ros::Subscriber sub;

    std::string current_topic_;
};

}

CSAPEX_REGISTER_CLASS(csapex::RosTrigger, csapex::Node)
