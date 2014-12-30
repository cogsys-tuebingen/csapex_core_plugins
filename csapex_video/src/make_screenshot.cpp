/// HEADER
#include "make_screenshot.h"

/// PROJECT
#include <csapex/signal/slot.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::MakeScreenshot, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


MakeScreenshot::MakeScreenshot()
{
}

void MakeScreenshot::setupParameters()
{
    addParameter(param::ParameterFactory::declareDirectoryOutputPath("path", "/tmp"));
    addParameter(param::ParameterFactory::declareRange<int>("quality", 0, 100, 75, 1));
    addParameter(param::ParameterFactory::declareText("format", "%Y-%m-%d_$wx$h.png"));
}

void MakeScreenshot::setup()
{
    in_ = modifier_->addSlot("Trigger", boost::bind(&MakeScreenshot::makeScreenshot, this));

    modifier_->setIsSource(true);
    modifier_->setIsSink(true);
}

void MakeScreenshot::process()
{
}

void MakeScreenshot::makeScreenshot()
{
    std::string path = readParameter<std::string>("path");
    std::stringstream ss;
    ss << "cd " << path << " && scrot ";
    ss << '\'' << readParameter<std::string>("format") << '\'';
    ss << " -q " << readParameter<int>("quality");

    system(ss.str().c_str());
}

