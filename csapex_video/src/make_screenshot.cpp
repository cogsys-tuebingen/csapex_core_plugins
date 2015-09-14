/// HEADER
#include "make_screenshot.h"

/// PROJECT
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::MakeScreenshot, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


MakeScreenshot::MakeScreenshot()
{
}

void MakeScreenshot::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareDirectoryOutputPath("path", "/tmp"));
    parameters.addParameter(param::ParameterFactory::declareRange<int>("quality", 0, 100, 75, 1));
    parameters.addParameter(param::ParameterFactory::declareText("format", "%Y-%m-%d_$wx$h.png"));
}

void MakeScreenshot::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addSlot("Trigger", std::bind(&MakeScreenshot::makeScreenshot, this));
    done_ = node_modifier.addTrigger("Done");

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

    if(system(ss.str().c_str())) {
        aerr << "call to " << ss.str() << " failed" << std::endl;
    }


    done_->trigger();
}

