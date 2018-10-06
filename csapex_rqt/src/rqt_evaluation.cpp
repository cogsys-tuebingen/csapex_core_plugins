/// HEADER
#include "rqt_evaluation.h"

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/core/settings.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/error_handling.h>
#include <csapex/utility/exceptions.h>
#include <csapex/utility/thread.h>
#include <csapex/view/csapex_view_core.h>
#include <csapex/view/csapex_window.h>
#include <csapex/view/gui_exception_handler.h>

/// SYSTEM
#include <QBoxLayout>
#include <QMenuBar>
#include <QStringList>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(csapex_rqt, CsApex, csapex_rqt::CsApex, rqt_gui_cpp::Plugin)

using namespace csapex_rqt;
using namespace csapex;

CsApex::CsApex() : handler(false)
{
}

CsApex::~CsApex()
{
    //    delete window;
}

void CsApex::initPlugin(qt_gui_cpp::PluginContext& context)
{
    context_ = &context;

    core.reset(new CsApexCore(settings_, handler));
    view_core.reset(new CsApexViewCore(*core));

    window = new CsApexWindow(*view_core);
    //    eva_->showMenu();

    context_->addWidget(window);

    window->start();
    core->startup();
    window->show();
}

void CsApex::shutdownPlugin()
{
    window->close();
    csapex::error_handling::stop_request()();
}

void CsApex::saveSettings(qt_gui_cpp::Settings& /*plugin_settings*/, qt_gui_cpp::Settings& instance_settings) const
{
    instance_settings.setValue("file", core->getSettings().get<std::string>("config").c_str());
}

void CsApex::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    QString file = instance_settings.value("file").toString();
    if (!file.isEmpty()) {
        core->getSettings().set("config", file.toStdString());
        window->reload();
    }
}
