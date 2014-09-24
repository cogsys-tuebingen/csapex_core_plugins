/// HEADER
#include "export_jann_format.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
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

CSAPEX_REGISTER_CLASS(csapex::ExportJANNFormat, csapex::Node)

using namespace csapex;
using namespace connection_types;

ExportJANNFormat::ExportJANNFormat() :
    last_classes_(0),
    last_id_(-1),
    suffix_(0)
{
    suffix_ = 0;
}

void ExportJANNFormat::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("classes",
                                                       param::ParameterDescription("Amount of classes that are available."),
                                                       1, 255, 1, 1));

    addParameter(param::ParameterFactory::declareRange("current id",
                                                       param::ParameterDescription("Class of the amount of possible classes."),
                                                       1, 255, 1, 1));


    addParameter(param::ParameterFactory::declareText("filename",
                                                      param::ParameterDescription("Base name of the exported messages, suffixed by a counter"),
                                                      "msg"), boost::bind(&ExportJANNFormat::setExportPath, this));

    addParameter(param::ParameterFactory::declareDirectoryOutputPath("path",
                                                                     param::ParameterDescription("Directory to write messages to"),
                                                                     "", ""), boost::bind(&ExportJANNFormat::setExportPath, this));
}

void ExportJANNFormat::setup()
{
    connector_ = modifier_->addInput<FeaturesMessage>("Feature");
}

void ExportJANNFormat::setExportPath()
{
    path_ = readParameter<std::string>("path");
    base_ = readParameter<std::string>("filename");
}

void ExportJANNFormat::process()
{
    int classes     = readParameter<int>("classes");
    int current_id  = readParameter<int>("current id");

    if(classes != last_classes_) {
        id_label_.resize(classes, 0);
        last_classes_ = classes;
        param::RangeParameter::Ptr r =
                boost::dynamic_pointer_cast<param::RangeParameter>(getParameter("current id"));
        r->setMax(classes - 1);
    }

    if(current_id != last_id_) {
        id_label_.at(last_id_)   = 0;
        id_label_.at(current_id) = 1;
        last_id_ = current_id;
    }

    if(path_.empty()) {
        return;
    }

    FeaturesMessage::Ptr msg = connector_->getMessage<FeaturesMessage>();
    doExport(msg);
}

namespace {
template<typename T>
inline void exportVector(const std::vector<T> &vector,
                         std::ofstream &out)
{
    for(typename std::vector<T>::const_iterator
        it  = vector.begin() ;
        it != vector.end() ;
        ++it) {
        out << " " << *it;
    }
    out << std::endl;
}

inline void exportLabel(const std::vector<int> &label,
                        std::ofstream &out)
{
    for(std::vector<int>::const_iterator
        it = label.begin() ;
        it != label.end() ;
        ++it) {
        out << " " << *it;
    }
    out << std::endl;
}
}


void ExportJANNFormat::doExport(const FeaturesMessage::Ptr& msg)
{
    QDir dir(path_.c_str());
    if(!dir.exists()) {
        QDir().mkdir(path_.c_str());
    }

    std::stringstream ss;
    ss << "_" << suffix_;

    std::string file = path_ + "/" + base_ + ss.str().c_str() + Settings::message_extension;

    std::ofstream out(file.c_str());
      exportVector<float>(msg->value, out);
      exportLabel(id_label_, out);
    out.close();

    ++suffix_;
}
