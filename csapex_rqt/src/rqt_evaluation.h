#ifndef RQT_EVALUATION_H
#define RQT_EVALUATION_H

/// PROJECT
#include <csapex/view/csapex_window.h>
#include <csapex/core/drag_io.h>

/// SYSTEM
#include <rqt_gui_cpp/plugin.h>

namespace csapex_rqt {

class CsApex : public rqt_gui_cpp::Plugin
{
public:
    CsApex();
    virtual ~CsApex();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
    qt_gui_cpp::PluginContext* context_;

    csapex::Settings settings_;
    csapex::GraphPtr graph_;
    csapex::GraphWorkerPtr graph_worker_;
    csapex::NodeFactoryPtr node_factory_;
    csapex::NodeAdapterFactoryPtr node_adapter_factory_;
    csapex::WidgetControllerPtr widget_controller_;
    csapex::CommandDispatcher::Ptr dispatcher_;
    csapex::CsApexCore core_;
    csapex::ThreadPoolPtr thread_pool_;
    csapex::CsApexWindow* eva_;


    csapex::DragIO drag_io_;
    csapex::DesignerScene* scene_;
    csapex::DesignerView* view_;
    csapex::Designer* designer_;
};

}

#endif // RQT_EVALUATION_H
