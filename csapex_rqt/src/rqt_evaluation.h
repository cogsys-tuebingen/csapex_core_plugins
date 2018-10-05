#ifndef RQT_EVALUATION_H
#define RQT_EVALUATION_H

/// PROJECT
#include <csapex/command/command_fwd.h>
#include <csapex/core/csapex_core.h>
#include <csapex/core/exception_handler.h>
#include <csapex/core/settings.h>
#include <csapex/model/observer.h>
#include <csapex/utility/exceptions.h>
#include <csapex/view/csapex_window.h>

/// SYSTEM
#include <rqt_gui_cpp/plugin.h>

namespace csapex_rqt
{
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
    csapex::ExceptionHandler handler;

    csapex::CsApexCorePtr core;

    std::shared_ptr<csapex::CsApexViewCore> view_core;
    csapex::CsApexWindow* window;
};

}  // namespace csapex_rqt

#endif  // RQT_EVALUATION_H
