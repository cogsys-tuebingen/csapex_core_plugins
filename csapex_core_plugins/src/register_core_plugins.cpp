/// HEADER
#include "register_core_plugins.h"

/// COMPONENT
#include "file_importer.h"

/// PROJECT
#include <csapex/msg/message_factory.h>
#include <csapex/model/tag.h>
#include <csapex/core/drag_io.h>
#include <csapex/command/add_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_state.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <boost/bind.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <QStringList>
#include <QUrl>

CSAPEX_REGISTER_CLASS(csapex::RegisterCorePlugins, csapex::CorePlugin)

using namespace csapex;

namespace {

class FileHandler
        : public DragIO::HandlerEnter, public DragIO::HandlerMove, public DragIO::HandlerDrop
{
    static const std::string format;

    QList<QUrl> getFiles(QDropEvent* e)
    {
        QByteArray itemData = e->mimeData()->data(format.c_str());
        QDataStream stream(&itemData, QIODevice::ReadOnly);

        int r, c;
        QMap<int, QVariant> v;
        stream >> r >> c >> v;

        e->accept();

        return e->mimeData()->urls();
    }

    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, QDragEnterEvent* e) {
        if(e->mimeData()->hasFormat(format.c_str())) {
            QList<QUrl> files = getFiles(e);

            if(files.size() != 0) {
                e->accept();
                return true;
            }
        }
        return false;
    }
    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, QDragMoveEvent* e){
        return false;
    }
    virtual bool handle(CommandDispatcher* dispatcher, QWidget *src, QDropEvent* e) {
        if(e->mimeData()->hasFormat(format.c_str())) {
            QList<QUrl> files = getFiles(e);

            if(files.size() == 0) {
                return false;
            }

            if(files.size() > 1) {
                std::cerr << "warning: droppend more than one file, using the first one" << std::endl;
            }

            std::cout << "file: " << files.first().toString().toStdString() << std::endl;
            QFile file(files.first().toLocalFile());
            if(file.exists()) {
                QPoint pos = e->pos();

                UUID uuid = UUID::make(dispatcher->getGraph()->makeUUIDPrefix("csapex::FileImporter"));


                NodeState::Ptr state(new NodeState(NULL));
                GenericState::Ptr child_state(new GenericState);
                child_state->addParameter(param::ParameterFactory::declareFileInputPath("path", files.first().toString().toStdString()));
                state->setChildState(child_state);

                std::string type("csapex::FileImporter");
                dispatcher->execute(Command::Ptr(new command::AddNode(type, pos, UUID::NONE, uuid, state)));

                e->accept();
                return true;
            }
        }
        return false;
    }
};

const std::string FileHandler::format = "text/uri-list";

}

RegisterCorePlugins::RegisterCorePlugins()
{
}

void RegisterCorePlugins::initUI(DragIO &dragio)
{
    Tag::createIfNotExists("Buffer");
    Tag::createIfNotExists("General");
    Tag::createIfNotExists("Input");
    Tag::createIfNotExists("Output");
    Tag::createIfNotExists("RosIO");
    Tag::createIfNotExists("ConsoleIO");
    Tag::createIfNotExists("Debug");

    dragio.registerHandler<FileHandler>();
}

void RegisterCorePlugins::init(CsApexCore& core)
{
}

void RegisterCorePlugins::shutdown()
{
}
