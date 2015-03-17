/// HEADER
#include "arff_format_export.h"

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

CSAPEX_REGISTER_CLASS(csapex::ArffFormatExport, csapex::Node)

using namespace csapex;
using namespace connection_types;

ArffFormatExport::ArffFormatExport()
{
}

void ArffFormatExport::setupParameters(Parameterizable& parameters)
{
    addParameter(param::ParameterFactory::
                 declareFileOutputPath("path",
                                       param::ParameterDescription("Directory to write messages to"),
                                       "", ".arff"));

    addParameter(param::ParameterFactory::
                 declareTrigger("save",
                                param::ParameterDescription("Save the obtained data!")),
                 std::bind(&ArffFormatExport::save, this));

    addParameter(param::ParameterFactory::
                 declareTrigger("clear",
                                param::ParameterDescription("Clear buffered data!")),
                 std::bind(&ArffFormatExport::clear, this));

    addParameter(param::ParameterFactory::
                 declareText("relation",
                             param::ParameterDescription("Define a relation title for the arff format."),
                             ""));
}

void ArffFormatExport::setup(NodeModifier& node_modifier)
{
    in_        = node_modifier.addOptionalInput<FeaturesMessage>("Feature");
    in_vector_ = node_modifier.addOptionalInput<GenericVectorMessage, FeaturesMessage>("Features");
}

void ArffFormatExport::process()
{
    if(msg::hasMessage(in_)) {
        FeaturesMessage::ConstPtr msg = msg::getMessage<FeaturesMessage>(in_);
        m_.lock();
        msgs_classes_.insert(msg->classification);
        msgs_.push_back(*msg);
        m_.unlock();
    }

    if(msg::hasMessage(in_vector_)) {
        std::shared_ptr<std::vector<FeaturesMessage> const> msgs =
                   msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_vector_);
        m_.lock();
        for(std::vector<FeaturesMessage>::const_iterator
            it = msgs->begin() ;
            it != msgs->end();
            ++it) {
            msgs_classes_.insert(it->classification);
            msgs_.push_back(*it);
        }
        m_.unlock();
    }
}

namespace {
const static std::string TAG_RELATION("@relation");
const static std::string TAG_ATTRIBUTE("@attribute attr_");
const static std::string TYPE_ATTRIBUTE("numeric");
const static std::string TAG_CLASS("@attribute class");
const static std::string TAG_DATA("@data");
const static std::string CLASS_PREFIX("cl_");

inline void writeHeader(const std::string   &relation,
                        const unsigned int   features,
                        const std::set<int> &classes,
                        std::ofstream       &out)
{
    out << TAG_RELATION  << " " << relation << std::endl;
    for(unsigned int i = 0 ; i < features ; ++i)
        out << TAG_ATTRIBUTE  << i << " " << TYPE_ATTRIBUTE << std::endl;
    out << TAG_CLASS << " { ";
    auto it_class = classes.begin();
    for(unsigned int i = 0 ; i < classes.size() -1 ; ++i, ++it_class)
        out << CLASS_PREFIX <<  *it_class << " , ";

    out << CLASS_PREFIX << *(++it_class)  << " }" << std::endl;
    out << TAG_DATA << std::endl;
}

template<typename T>
inline void exportVector(const std::vector<T> &vector,
                        std::ofstream &out)
{
    for(typename std::vector<T>::const_iterator
        it  = vector.begin() ;
        it != vector.end() ;
        ++it) {
        out << *it << ", ";
    }
}
}


void ArffFormatExport::save()
{
    std::vector<FeaturesMessage> msgs;
    m_.lock();
    msgs = msgs_;
    m_.unlock();

    if(msgs.empty())
        return;

    std::string     path = readParameter<std::string>("path");
    std::string     relation = readParameter<std::string>("relation");
    if(relation == "")
        relation = "You should enter a relation, dude!";
    std::ofstream   out_file(path.c_str());

    writeHeader(relation,
                msgs.front().value.size(),
                msgs_classes_,
                out_file);

    for(auto it  = msgs.begin() ;
             it != msgs.end() ;
           ++it) {
        exportVector<float>(it->value, out_file);
        out_file << CLASS_PREFIX << it->classification << std::endl;
    }

    out_file.close();
}

void ArffFormatExport::clear()
{
    m_.lock();
    msgs_classes_.clear();
    msgs_.clear();
    m_.unlock();
}
