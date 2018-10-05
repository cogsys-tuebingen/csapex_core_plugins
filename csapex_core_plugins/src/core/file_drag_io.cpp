/// PROJECT
#include <csapex/command/add_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/model/generic_state.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node_state.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/export_plugin.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/utility/uuid.h>
#include <csapex/view/designer/drag_io_handler.h>
#include <csapex/view/designer/graph_view.h>

/// SYSTEM
#include <QDir>
#include <QFile>
#include <QMimeData>
#include <iostream>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN FileHandler : public DragIOHandler
{
    bool handleEnter(GraphView* view, CommandExecutor* dispatcher, QDragEnterEvent* e) override
    {
        if (e->mimeData()->hasUrls()) {
            e->acceptProposedAction();
            return true;

        } else {
            return false;
        }
    }
    bool handleMove(GraphView* view, CommandExecutor* dispatcher, QDragMoveEvent* e) override
    {
        if (e->mimeData()->hasUrls()) {
            e->acceptProposedAction();
            return true;

        } else {
            return false;
        }
    }
    bool handleDrop(GraphView* view, CommandExecutor* dispatcher, QDropEvent* e, const QPointF& scene_pos) override
    {
        if (e->mimeData()->hasUrls()) {
            QList<QUrl> files = e->mimeData()->urls();

            if (files.size() == 0) {
                return false;
            }

            if (files.size() > 1) {
                std::cerr << "warning: droppend more than one file, using the first one" << std::endl;
            }

            std::cout << "file: " << files.first().toString().toStdString() << std::endl;

            QString file_str = files.first().toLocalFile();
            QFile file(file_str);

            if (file.exists()) {
                GraphFacade* gf = view->getGraphFacade();
                UUID uuid = gf->generateUUID("csapex::FileImporter");

                NodeState::Ptr state(new NodeState(nullptr));
                GenericState::Ptr child_state(new GenericState);

                QDir dir(file_str);
                child_state->addParameter(csapex::param::factory::declareBool("import directory", dir.exists()));
                if (dir.exists()) {
                    child_state->addParameter(csapex::param::factory::declareFileInputPath("directory", file_str.toStdString()));
                } else {
                    child_state->addParameter(csapex::param::factory::declareFileInputPath("path", file_str.toStdString()));
                }
                state->setParameterState(child_state);

                std::string type("csapex::FileImporter");
                dispatcher->executeLater(Command::Ptr(new command::AddNode(gf->getAbsoluteUUID(), type, Point(scene_pos.x(), scene_pos.y()), uuid, state)));

                e->accept();
                return true;
            }
        }
        return false;
    }
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::FileHandler, csapex::DragIOHandler)
