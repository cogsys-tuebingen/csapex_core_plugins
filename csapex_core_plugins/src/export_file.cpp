/// HEADER
#include "export_file.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/message.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/path_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/core/settings.h>
#include <csapex/model/message_factory.h>

/// SYSTEM
#include <QFileDialog>
#include <fstream>

CSAPEX_REGISTER_CLASS(csapex::ExportFile, csapex::Node)

using namespace csapex;

ExportFile::ExportFile()
{
    addTag(Tag::get("Output"));
    addTag(Tag::get("General"));

    suffix_ = 0;
}

void ExportFile::setupParameters()
{
    addParameter(param::ParameterFactory::declareBool("yaml",
                                                      param::ParameterDescription("Export message in cs::APEX-YAML format?"),
                                                      false));
    addParameter(param::ParameterFactory::declareText("filename",
                                                      param::ParameterDescription("Base name of the exported messages, suffixed by a counter"),
                                                      "msg"), boost::bind(&ExportFile::setExportPath, this));
    addParameter(param::ParameterFactory::declareDirectoryOutputPath("path",
                                                                     param::ParameterDescription("Directory to write messages to"),
                                                                     "", ""), boost::bind(&ExportFile::setExportPath, this));
}

void ExportFile::setup()
{
    connector_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
}

void ExportFile::setExportPath()
{
    path_ = readParameter<std::string>("path");
    base_ = readParameter<std::string>("filename");
}

void ExportFile::process()
{
    if(path_.empty()) {
        return;
    }

    ConnectionType::Ptr msg = connector_->getMessage<ConnectionType>();

    QDir dir(path_.c_str());
    if(!dir.exists()) {
        QDir().mkdir(path_.c_str());
    }

    std::stringstream ss;
    ss << "_" << suffix_;

    if(readParameter<bool>("yaml")) {
        std::string file = path_ + "/" + base_ + ss.str().c_str() + Settings::message_extension;
        MessageFactory::writeMessage(file, msg);
    } else {
        msg->writeRaw(path_, ss.str());
    }

    ++suffix_;
}
