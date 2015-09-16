/// PROJECT
#include <csapex/view/designer/drag_io_handler.h>
#include <csapex/utility/uuid.h>
#include <csapex/command/dispatcher.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_state.h>
#include <csapex/model/generic_state.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/command/add_node.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <QMimeData>
#include <iostream>
#include <boost/regex.h>

namespace csapex
{

class FileHandler : public DragIOHandler
{
    virtual bool handleEnter(CommandDispatcher* dispatcher, QWidget *src, QDragEnterEvent* e) {
        if(e->mimeData()->hasUrls()) {
            e->acceptProposedAction();
            return true;

        } else{
            return false;
        }
    }
    virtual bool handleMove(CommandDispatcher* dispatcher, QWidget *src, QDragMoveEvent* e){
        if(e->mimeData()->hasUrls()) {
            e->acceptProposedAction();
            return true;

        } else{
            return false;
        }
    }
    virtual bool handleDrop(CommandDispatcher* dispatcher, QWidget *src, QDropEvent* e, const QPointF& scene_pos) {
        if(e->mimeData()->hasUrls()) {
            QList<QUrl> files = e->mimeData()->urls();

            if(files.size() == 0) {
                return false;
            }

            if(files.size() > 1) {
                std::cerr << "warning: droppend more than one file, using the first one" << std::endl;
            }

            std::cout << "file: " << files.first().toString().toStdString() << std::endl;
            QFile file(files.first().toLocalFile());
            if(file.exists()) {
                UUID uuid = UUID::make(dispatcher->getGraph()->makeUUIDPrefix("csapex::FileImporter"));

                NodeState::Ptr state(new NodeState(nullptr));
                GenericState::Ptr child_state(new GenericState);
                child_state->addParameter(csapex::param::ParameterFactory::declareFileInputPath("path", files.first().toLocalFile().toStdString()));
                state->setParameterState(child_state);

                std::string type("csapex::FileImporter");
                dispatcher->execute(Command::Ptr(new command::AddNode(type, Point(scene_pos.x(), scene_pos.y()), UUID::NONE, uuid, state)));

                e->accept();
                return true;
            }
        }
        return false;
    }
};

}

CSAPEX_REGISTER_CLASS(csapex::FileHandler, csapex::DragIOHandler)
