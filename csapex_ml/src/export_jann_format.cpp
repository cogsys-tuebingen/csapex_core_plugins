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
    save_(false),
    clear_(false)
{
}

void ExportJANNFormat::setupParameters()
{

    addParameter(param::ParameterFactory::declarePath("path",
                                                      param::ParameterDescription("Directory to write messages to"),
                                                      "", ""));

    addParameter(param::ParameterFactory::declareBool("relabel", false));

    addParameter(param::ParameterFactory::declareTrigger("save",
                                                         param::ParameterDescription("Save the obtained data!")),
                                                         boost::bind(&ExportJANNFormat::requestSave, this));
    addParameter(param::ParameterFactory::declareTrigger("clear",
                                                         param::ParameterDescription("Clear buffered data!")),
                                                         boost::bind(&ExportJANNFormat::requestClear, this));
}

void ExportJANNFormat::setup()
{
    connector_ = modifier_->addInput<FeaturesMessage>("Feature");
}

void ExportJANNFormat::process()
{
    if(clear_) {
        msgs_.clear();
        clear_ = false;
    }

    if(save_) {
        doExport();
        msgs_.clear();
        save_ = false;
    }

    FeaturesMessage::Ptr msg = connector_->getMessage<FeaturesMessage>();
    msgs_.push_back(msg);
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

inline void labelMap(const std::vector<FeaturesMessage::Ptr> &msgs,
                     std::map<int, std::vector<int> > &labels)
{
    for(std::vector<FeaturesMessage::Ptr>::const_iterator
        it = msgs.begin() ;
        it != msgs.end() ;
        ++it) {

        int class_id = (*it)->classification;
        if(labels.find(class_id) == labels.end()) {
            labels.insert(std::make_pair(class_id, std::vector<int>()));
        }
    }

    int class_count = labels.size();
    int current_idx = 0;
    for(std::map<int, std::vector<int> >::iterator
        it  = labels.begin() ;
        it != labels.end() ;
        ++it ) {
        it->second.resize(class_count, 0);
        it->second.at(current_idx) = 1;
    }
}
}


void ExportJANNFormat::doExport()
{
    std::string path = readParameter<std::string>("path");
    std::ofstream out(path.c_str());

    std::map<int, std::vector<int> > labels;
    labelMap(msgs_, labels);

    for(std::vector<FeaturesMessage::Ptr>::iterator
        it  = msgs_.begin() ;
        it != msgs_.end() ;
        ++it) {
        exportVector<float>((*it)->value, out);
        exportVector<int>(labels.at((*it)->classification), out);
    }

    out.close();
}

void ExportJANNFormat::requestSave()
{
    save_ = true;
}

void ExportJANNFormat::requestClear()
{
    clear_ = true;
}
