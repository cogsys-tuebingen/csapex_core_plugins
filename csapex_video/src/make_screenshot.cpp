/// HEADER
#include "make_screenshot.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::MakeScreenshot, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

MakeScreenshot::MakeScreenshot()
{
}

void MakeScreenshot::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareDirectoryOutputPath("path", "/tmp"));
    parameters.addParameter(csapex::param::factory::declareRange<int>("quality", 0, 100, 75, 1));
    parameters.addParameter(csapex::param::factory::declareText("format", "%s_$wx$h.png"));
}

void MakeScreenshot::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addSlot("Trigger", std::bind(&MakeScreenshot::makeScreenshot, this));
    done_ = node_modifier.addEvent("Done");
}

void MakeScreenshot::process()
{
}

void MakeScreenshot::makeScreenshot()
{
    std::string path = readParameter<std::string>("path");
    std::stringstream ss;
    ss << "mkdir -p " << path << " && ";
    ss << "cd " << path << " && scrot ";
    ss << '\'' << readParameter<std::string>("format") << '\'';
    ss << " -q " << readParameter<int>("quality");
    if (system(ss.str().c_str())) {
        throw std::runtime_error(std::string("call to ") + ss.str() + " failed");
    }

    msg::trigger(done_);
}
