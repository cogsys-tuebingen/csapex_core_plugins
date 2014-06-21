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

/// SYSTEM
#include <boost/regex.hpp>

CSAPEX_REGISTER_CLASS(csapex::APEXRosInterface, csapex::CorePlugin)

using namespace csapex;


namespace {
class RosHandler
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
    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, QDropEvent* e) {
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
                    state->setChildState(dummy.getChildState());

                    std::string type("csapex::ImportRos");
                    dispatcher->execute(Command::Ptr(new command::AddNode(type, pos, UUID::NONE, uuid, state)));

                    return true;
                }
            }
        }
        return false;
    }
};

const boost::regex RosHandler::fmt("[a-zA-Z0-9_\\-/]+");
}

APEXRosInterface::APEXRosInterface()
    : core_(NULL)
{
}

APEXRosInterface::~APEXRosInterface()
{
}

void APEXRosInterface::init(CsApexCore &core)
{
    core_ = &core;

    ROSHandler::instance().waitForConnection();

    if(ROSHandler::instance().isConnected()) {
        std::cerr << "subscribing to /syscommand" << std::endl;
        command_sub_ = ROSHandler::instance().nh()->subscribe
                <std_msgs::String>("/syscommand", 10, boost::bind(&APEXRosInterface::command, this, _1));

        ros::spinOnce();

    } else {
        std::cerr << "cannot init ros interface, no ros handler" << std::endl;
    }
}

void APEXRosInterface::initUI(DragIO &dragio)
{
    dragio.registerHandler<RosHandler>();
}

void APEXRosInterface::command(const std_msgs::StringConstPtr& cmd)
{
    std::string command = cmd->data;

    if(command == "pause") {
        core_->setPause(true);

    } else if(command == "unpause" || command == "continue" || command == "play"){
        core_->setPause(false);
    }
}

void APEXRosInterface::shutdown()
{
    ROSHandler::instance().stop();
}
