/// HEADER
#include "apex_ros_interface.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ros/ros_handler.h>
#include "import_ros.h"
#include <csapex/core/drag_io.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_state.h>
#include <csapex/command/add_node.h>
#include <csapex/msg/message_factory.h>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_ros/yaml_io.hpp>

/// SYSTEM
#include <boost/regex.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

CSAPEX_REGISTER_CLASS(csapex::APEXRosInterface, csapex::CorePlugin)

using namespace csapex;


namespace {
class RosIoHandler
        : public DragIO::HandlerEnter, public DragIO::HandlerMove, public DragIO::HandlerDrop
{
    static const boost::regex fmt;

    std::string getCmd(QDropEvent* e)
    {
        QByteArray itemData = e->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&itemData, QIODevice::ReadOnly);

        int r, c;
        QMap<int, QVariant> v;
        stream >> r >> c >> v;

        return v[Qt::UserRole].toString().toStdString();
    }

    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, QDragEnterEvent* e) {
        if(e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
            std::string cmd = getCmd(e);

            bool cmd_could_be_topic = boost::regex_match(cmd, fmt);

            if(cmd_could_be_topic) {
                if(ROSHandler::instance().isConnected() && ROSHandler::instance().topicExists(cmd)) {
                    e->accept();
                    return true;
                }
            }
        }
        return false;
    }
    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, QDragMoveEvent* e){
        return false;
    }
    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, QDropEvent* e, const QPointF& scene_pos) {
        if(e->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist")) {
            std::string cmd = getCmd(e);

            bool cmd_could_be_topic = boost::regex_match(cmd, fmt);

            if(cmd_could_be_topic) {
                if(ROSHandler::instance().topicExists(cmd)) {
                    QPoint pos = e->pos();

                    UUID uuid = UUID::make(dispatcher->getGraph()->makeUUIDPrefix("csapex::ImportRos"));

                    NodeState::Ptr state(new NodeState(NULL));
                    ImportRos dummy;
                    dummy.getParameter("topic")->set(cmd);
                    state->setParameterState(dummy.getParameterState());

                    std::string type("csapex::ImportRos");
                    dispatcher->execute(Command::Ptr(new command::AddNode(type, pos, UUID::NONE, uuid, state)));

                    return true;
                }
            }
        }
        return false;
    }
};

const boost::regex RosIoHandler::fmt("[a-zA-Z0-9_\\-/]+");
}


template <typename RosType, typename ApexType>
struct ConvertIntegral
{
    static typename connection_types::GenericValueMessage<ApexType>::Ptr ros2apex(const typename RosType::ConstPtr &ros_msg) {
        typename connection_types::GenericValueMessage<ApexType>::Ptr out(new connection_types::GenericValueMessage<ApexType>);
        out->value = ros_msg->data;
        return out;
    }
    static typename RosType::Ptr apex2ros(const typename connection_types::GenericValueMessage<ApexType>::Ptr& apex_msg) {
        typename RosType::Ptr out(new RosType);
        out->data = apex_msg->value;
        return out;
    }
};




APEXRosInterface::APEXRosInterface()
    : core_(NULL), disabled_(false)
{
}

APEXRosInterface::~APEXRosInterface()
{
}

void APEXRosInterface::prepare(Settings &settings)
{
    ROSHandler::createInstance(settings);
}

void APEXRosInterface::init(CsApexCore &core)
{
    core_ = &core;

    ROSHandler::instance().registerConnectionCallback(boost::bind(&APEXRosInterface::registerCommandListener, this));

    RosMessageConversion::registerConversion<std_msgs::Bool, connection_types::GenericValueMessage<bool>, ConvertIntegral<std_msgs::Bool, bool> >();
    RosMessageConversion::registerConversion<std_msgs::Int32, connection_types::GenericValueMessage<int>, ConvertIntegral<std_msgs::Int32, int> >();
    RosMessageConversion::registerConversion<std_msgs::Float64, connection_types::GenericValueMessage<double>, ConvertIntegral<std_msgs::Float64, double> >();
    RosMessageConversion::registerConversion<std_msgs::String, connection_types::GenericValueMessage<std::string>, ConvertIntegral<std_msgs::String, std::string> >();

    RosMessageConversionT<nav_msgs::Odometry>::registerConversion();
    RosMessageConversionT<geometry_msgs::PoseStamped>::registerConversion();    
    RosMessageConversionT<visualization_msgs::MarkerArray>::registerConversion();
}

void APEXRosInterface::registerCommandListener()
{
    assert(ROSHandler::instance().isConnected());
    global_command_sub_ = ROSHandler::instance().nh()->subscribe
            <std_msgs::String>("/syscommand", 10, boost::bind(&APEXRosInterface::command, this, _1, true));
    private_command_sub_ = ROSHandler::instance().nh()->subscribe
            <std_msgs::String>("command", 10, boost::bind(&APEXRosInterface::command, this, _1, false));

    ros::spinOnce();
}

void APEXRosInterface::initUI(DragIO &dragio)
{
    dragio.registerHandler<RosIoHandler>();
}

void APEXRosInterface::command(const std_msgs::StringConstPtr& cmd, bool global_cmd)
{
    std::string command = cmd->data;
    bool local_cmd = !global_cmd;

    /*
     * pause / unpause:
     *   - temporary disable everything to conserve computational power
     *   - globally accepted, if this node is not disabled
     *
     * stop / resume
     *   - completely disable / enable this subsystem, when it is not needed
     *   - only locally accepted
     *   - overwrites pause -> a disabled instance cannot be unpaused
     */

    if(!disabled_) {
        // disabled state is stronger than pause / unpause
        if(command == "pause") {
            core_->setPause(true);

        } else if(command == "unpause"){
            core_->setPause(false);
        }
    }

    if(local_cmd) {
        if(command == "stop") {
            disabled_ = true;
            core_->setPause(true);
        } else if(command == "resume") {
            disabled_ = false;
            core_->setPause(false);
        }
    }
}

void APEXRosInterface::shutdown()
{
    ROSHandler::instance().stop();
}
