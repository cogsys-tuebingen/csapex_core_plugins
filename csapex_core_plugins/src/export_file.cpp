/// HEADER
#include "export_file.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/connection_type.h>
#include <csapex/msg/message.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/path_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/core/settings.h>
#include <csapex/msg/message_factory.h>

/// SYSTEM
#include <QFileDialog>
#include <fstream>

CSAPEX_REGISTER_CLASS(csapex::ExportFile, csapex::Node)

using namespace csapex;

ExportFile::ExportFile()
{
    suffix_ = 0;
}

void ExportFile::setupParameters(Parameterizable& parameters)
{
    addParameter(param::ParameterFactory::declareBool("yaml",
                                                      param::ParameterDescription("Export message in cs::APEX-YAML format?"),
                                                      false));
    addParameter(param::ParameterFactory::declareText("filename",
                                                      param::ParameterDescription("Base name of the exported messages, suffixed by a counter"),
                                                      "msg"), std::bind(&ExportFile::setExportPath, this));
    addParameter(param::ParameterFactory::declareDirectoryOutputPath("path",
                                                                     param::ParameterDescription("Directory to write messages to"),
                                                                     "", ""), std::bind(&ExportFile::setExportPath, this));
}

void ExportFile::setup(NodeModifier& node_modifier)
{
    connector_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
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

    ConnectionType::ConstPtr msg = msg::getMessage<ConnectionType>(connector_);
    connection_types::VectorMessage::ConstPtr vector = std::dynamic_pointer_cast<const connection_types::VectorMessage>(msg);
    if(vector) {
        exportVector(vector);
    } else {
        exportSingle(msg);
    }
}

void ExportFile::exportVector(const connection_types::VectorMessage::ConstPtr& vector)
{
    for(std::size_t i = 0, total = vector->value.size(); i < total; ++i) {
        exportSingle(vector->value[i]);
    }
}

void ExportFile::exportSingle(const ConnectionType::ConstPtr& msg)
{
    QDir dir(path_.c_str());
    if(!dir.exists()) {
        QDir().mkdir(path_.c_str());
    }



    if(readParameter<bool>("yaml")) {
        while(true) {
            std::stringstream file_s;
            file_s << path_ << "/" << base_ << "_" << suffix_ << Settings::message_extension;
            std::string file = file_s.str();

            if(!QFile(QString::fromStdString(file)).exists()) {
                MessageFactory::writeMessage(file, *msg);
                break;
            } else {
                ++suffix_;
            }
        }
    } else {
        std::stringstream ss;
        ss << "_" << suffix_;
        msg->writeRaw(path_, base_, ss.str());
    }

    ++suffix_;
}
