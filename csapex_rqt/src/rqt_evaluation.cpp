/// HEADER
#include "rqt_evaluation.h"

/// PROJECT
#include <csapex/msg/message_factory.h>
#include <csapex/view/designer.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/core/graphio.h>
#include <csapex/view/designer_view.h>
#include <csapex/view/designer.h>
#include <csapex/core/settings.h>
#include <csapex/model/node_factory.h>
#include <csapex/view/node_adapter_factory.h>
#include <csapex/view/widget_controller.h>
#include <csapex/view/designer_scene.h>
#include <csapex/core/thread_pool.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QMenuBar>
#include <QBoxLayout>

PLUGINLIB_DECLARE_CLASS(csapex_rqt, CsApex, csapex_rqt::CsApex, rqt_gui_cpp::Plugin)

using namespace csapex_rqt;
using namespace csapex;

CsApex::CsApex()
    : graph_(new Graph),
      graph_worker_(new GraphWorker(&settings_, graph_.get())),
      node_factory_(new NodeFactory(settings_)),
      node_adapter_factory_(new NodeAdapterFactory(settings_)),
      widget_controller_(new csapex::WidgetController(settings_, graph_, node_factory_.get(), node_adapter_factory_.get())),
      dispatcher_(new CommandDispatcher(settings_, graph_worker_, widget_controller_)),
      core_(settings_, graph_worker_, node_factory_.get(), node_adapter_factory_.get(), dispatcher_.get()),
      thread_pool_(new ThreadPool(&core_, graph_.get(), true, false)),
      drag_io_(graph_.get(), dispatcher_.get(), widget_controller_),
      scene_(new DesignerScene(graph_, dispatcher_.get(), widget_controller_)),
      view_ (new DesignerView(scene_, graph_, settings_, *thread_pool_, dispatcher_.get(), widget_controller_, drag_io_)),
      designer_(new Designer(settings_, graph_, dispatcher_.get(), widget_controller_, view_, scene_))
{
    widget_controller_->setDesigner(designer_);
}

CsApex::~CsApex()
{
    delete eva_;
}


void CsApex::initPlugin(qt_gui_cpp::PluginContext& context)
{
    context_ = &context;

    eva_ = new CsApexWindow(core_, dispatcher_.get(), widget_controller_, graph_worker_, designer_);
//    eva_->showMenu();

    context_->addWidget(eva_);
}

void CsApex::shutdownPlugin()
{
}

void CsApex::saveSettings(qt_gui_cpp::Settings& /*plugin_settings*/, qt_gui_cpp::Settings& instance_settings) const
{
    instance_settings.setValue("file", core_.getSettings().get<std::string>("config").c_str());
}

void CsApex::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    QString file = instance_settings.value("file").toString();
    if(!file.isEmpty()) {
        core_.getSettings().set("config", file.toStdString());
        eva_->reload();
    }
}
