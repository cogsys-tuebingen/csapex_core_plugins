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

/// SYSTEM
#include <QFileDialog>

CSAPEX_REGISTER_CLASS(csapex::ExportFile, csapex::Node)

using namespace csapex;

ExportFile::ExportFile()
{
    addTag(Tag::get("Output"));
    addTag(Tag::get("General"));

    addParameter(param::ParameterFactory::declareDirectoryOutputPath("path", "", ""), boost::bind(&ExportFile::setExportPath, this));

    suffix_ = 0;
}

QIcon ExportFile::getIcon() const
{
    return QIcon(":/terminal.png");
}

void ExportFile::setup()
{
    connector_ = modifier_->addInput<connection_types::AnyMessage>("Anything");
}

void ExportFile::setExportPath()
{
    path_ = param<std::string>("path");
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
    msg->writeRaw(path_, ss.str());

    ++suffix_;
}
