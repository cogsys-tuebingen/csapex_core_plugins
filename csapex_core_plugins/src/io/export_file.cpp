/// HEADER
#include "export_file.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/path_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/core/settings.h>
#include <csapex/factory/message_factory.h>
#include <csapex/msg/marker_message.h>

/// SYSTEM
#include <fstream>
#include <QDir>

CSAPEX_REGISTER_CLASS(csapex::ExportFile, csapex::Node)

using namespace csapex;

ExportFile::ExportFile()
    : oneshot_(false), oneshot_allowed_(false)
{
    suffix_ = 0;
}

void ExportFile::setupParameters(Parameterizable& parameters)
{
    param::ParameterPtr one_by_one = csapex::param::ParameterFactory::declareBool(
                "export one",
                csapex::param::ParameterDescription("Export files one-by-one."),
                false);
    addParameter(one_by_one, oneshot_);
    std::function<bool()> is_one = [one_by_one](){
        return one_by_one->as<bool>();
    };

    addConditionalParameter(param::ParameterFactory::declareTrigger("save one message"), is_one, [this](param::Parameter* p) {
        oneshot_allowed_ = true;
    });
    addConditionalParameter(param::ParameterFactory::declareTrigger("save last message"), is_one, [this](param::Parameter* p) {
        if (last_message_)
            exportMessage(last_message_);
    });

    addParameter(csapex::param::ParameterFactory::declareBool("yaml",
                                                      csapex::param::ParameterDescription("Export message in cs::APEX-YAML format?"),
                                                      true));
    addParameter(csapex::param::ParameterFactory::declareText("filename",
                                                      csapex::param::ParameterDescription("Base name of the exported messages, suffixed by a counter"),
                                                      "msg"), std::bind(&ExportFile::setExportPath, this));
    addParameter(csapex::param::ParameterFactory::declareDirectoryOutputPath("path",
                                                                     csapex::param::ParameterDescription("Directory to write messages to"),
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

    TokenData::ConstPtr msg = msg::getMessage<TokenData>(connector_);
    last_message_ = msg;

    if(oneshot_) {
        if(!oneshot_allowed_) {
            return;
        } else {
            oneshot_allowed_ = false;
        }
    }

    exportMessage(msg);
}

void ExportFile::exportMessage(const TokenData::ConstPtr &msg)
{
    connection_types::GenericVectorMessage::ConstPtr vector = std::dynamic_pointer_cast<const connection_types::GenericVectorMessage>(msg);
    if(vector) {
        exportVector(vector);
    } else {
        exportSingle(msg);
    }
}

void ExportFile::exportVector(const connection_types::GenericVectorMessage::ConstPtr& vector)
{

    for(std::size_t i = 0, total = vector->nestedValueCount(); i < total; ++i) {
        exportSingle(vector->nestedValue(i));
    }
}

void ExportFile::exportSingle(const TokenData::ConstPtr& msg)
{
    if(std::dynamic_pointer_cast<connection_types::MarkerMessage const>(msg)) {
        return;
    }

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
