/// HEADER
#include "export_file.h"

/// PROJECT
#include <csapex/core/settings.h>
#include <csapex/factory/message_factory.h>
#include <csapex/model/generic_state.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_state.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/io.h>
#include <csapex/msg/marker_message.h>
#include <csapex/msg/message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/path_parameter.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <QDir>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <fstream>

CSAPEX_REGISTER_CLASS(csapex::ExportFile, csapex::Node)

using namespace csapex;

ExportFile::ExportFile() : oneshot_(false), oneshot_allowed_(false)
{
    suffix_ = 0;
}

void ExportFile::setupParameters(Parameterizable& parameters)
{
    param::ParameterPtr one_by_one = csapex::param::factory::declareBool("export one", csapex::param::ParameterDescription("Export files one-by-one."), false);
    addParameter(one_by_one, oneshot_);
    std::function<bool()> is_one = [one_by_one]() { return one_by_one->as<bool>(); };

    addConditionalParameter(param::factory::declareTrigger("save one message"), is_one, [this](param::Parameter* p) { oneshot_allowed_ = true; });
    addConditionalParameter(param::factory::declareTrigger("save last message"), is_one, [this](param::Parameter* p) {
        if (last_message_)
            exportMessage(last_message_);
    });

    std::map<std::string, serialization::Format> formats{ { "Native file format (partial support)", serialization::Format::NATIVE },
                                                          { "CS::APEX binary format", serialization::Format::APEX_BINARY },
                                                          { "CS::APEX yaml format", serialization::Format::APEX_YAML } };

    addParameter(csapex::param::factory::declareParameterSet("format",
                                                             csapex::param::ParameterDescription("Export format. (Native is only supported some "
                                                                                                 "messages, such as PNG for images)"),
                                                             formats, serialization::Format::APEX_BINARY),
                 target_format_);

    addParameter(csapex::param::factory::declareText("filename", csapex::param::ParameterDescription("Base name of the exported messages, suffixed by a counter"), "msg"),
                 std::bind(&ExportFile::setExportPath, this));
    addParameter(csapex::param::factory::declareDirectoryOutputPath("path", csapex::param::ParameterDescription("Directory to write messages to"), "", ""),
                 std::bind(&ExportFile::setExportPath, this));
    addParameter(csapex::param::factory::declareBool("vector_as_single_file", false), vector_as_single_file_);

    addConditionalParameter(csapex::param::factory::declareBool("compress", csapex::param::ParameterDescription("Compress File?"), false), vector_as_single_file_, compress_);

    getNodeHandle()->getNodeState()->getParameterState()->legacy_parameter_added.connect([this](const param::ParameterPtr& p) {
        if (p->name() == "yaml" && p->as<bool>()) {
            if (target_format_ != serialization::Format::APEX_YAML) {
                ainfo << "legacy yaml export" << std::endl;
                setParameter("format", serialization::Format::APEX_YAML);
                target_format_ = serialization::Format::APEX_YAML;
            }
        }
    });
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
    if (path_.empty()) {
        return;
    }

    TokenData::ConstPtr msg = msg::getMessage<TokenData>(connector_);
    last_message_ = msg;

    if (oneshot_) {
        if (!oneshot_allowed_) {
            return;
        } else {
            oneshot_allowed_ = false;
        }
    }

    exportMessage(msg);
}

void ExportFile::exportMessage(const TokenData::ConstPtr& msg)
{
    connection_types::GenericVectorMessage::ConstPtr vector = std::dynamic_pointer_cast<const connection_types::GenericVectorMessage>(msg);
    if (vector) {
        exportVector(vector);
    } else {
        exportSingle(msg);
    }
}

void ExportFile::exportVector(const connection_types::GenericVectorMessage::ConstPtr& vector)
{
    if (vector_as_single_file_) {
        while (true) {
            std::stringstream file_s;
            if (compress_) {
                file_s << path_ << "/" << base_ << "_" << suffix_ << Settings::message_extension_compressed;
            } else {
                file_s << path_ << "/" << base_ << "_" << suffix_ << Settings::message_extension;
            }
            std::string file = file_s.str();

            if (!QFile(QString::fromStdString(file)).exists()) {
                if (compress_) {
                    std::ofstream out(file, std::ios_base::out | std::ios_base::binary);
                    boost::iostreams::filtering_ostream zipped_out;
                    zipped_out.push(boost::iostreams::gzip_compressor());
                    zipped_out.push(out);
                    YAML::Emitter em;
                    em << MessageSerializer::instance().serializeYamlMessage(*vector);
                    zipped_out << em.c_str();
                    break;
                } else {
                    YAML::Emitter em;
                    em << MessageSerializer::instance().serializeYamlMessage(*vector);
                    std::ofstream out(file);
                    out << em.c_str();
                    out.close();
                    break;
                }
            } else {
                ++suffix_;
            }
        }
    } else {
        for (std::size_t i = 0, total = vector->nestedValueCount(); i < total; ++i) {
            exportSingle(vector->nestedValue(i));
        }
    }
}

void ExportFile::exportSingle(const TokenData::ConstPtr& msg)
{
    // do not export markers
    if (std::dynamic_pointer_cast<connection_types::MarkerMessage const>(msg)) {
        return;
    }

    // export the file
    suffix_ = MessageFactory::writeFile(path_, base_, suffix_, *msg, target_format_);
}
