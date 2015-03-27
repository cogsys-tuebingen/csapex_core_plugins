/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <ros/time.h>

using namespace csapex::connection_types;


namespace csapex
{

class StampDelayMeasurement : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Message");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
        Message::ConstPtr msg(msg::getMessage<Message>(in_));

        ros::Time stamp;
        stamp.fromNSec(msg->stamp_micro_seconds * 1e3);

        ros::Time now = ros::Time::now();

        ainfo << "Current time: " << now << "\tStamp: " << stamp << "\tDelay: " << (now - stamp).toNSec() * 1e-6 << " milliseconds" << std::endl;
    }

private:
    Input* in_;
    Output* out_;
};


}

CSAPEX_REGISTER_CLASS(csapex::StampDelayMeasurement, csapex::Node)


