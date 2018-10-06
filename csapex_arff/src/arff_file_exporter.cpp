/// HEADER
#include "arff_file_exporter.h"

/// PROJECT
#include <csapex/factory/message_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/path_parameter.h>
#include <csapex/utility/register_apex_plugin.h>

#include <cslibs_arff/arff_data.h>

/// SYSTEM
#include <fstream>

CSAPEX_REGISTER_CLASS(csapex::ARFFFileExporter, csapex::Node)

using namespace csapex;
using namespace connection_types;

ARFFFileExporter::ARFFFileExporter()
{
}

void ARFFFileExporter::setupParameters(Parameterizable& parameters)
{
    CollectionNode<FeaturesMessage>::setupParameters(parameters);

    parameters.addParameter(param::factory::declareFileOutputPath("path", csapex::param::ParameterDescription("Directory to write messages to"), "", ".arff"), path_);

    parameters.addParameter(param::ParameterFactory::declareText("relation name", csapex::param::ParameterDescription("Define a relation title for the arff format."), ""), relation_name_);
}

void ARFFFileExporter::setup(NodeModifier& node_modifier)
{
    CollectionNode<connection_types::FeaturesMessage>::setup(node_modifier);
}

bool ARFFFileExporter::processCollection(std::vector<connection_types::FeaturesMessage>& collection)
{
    if (collection.size() == 0)
        return false;

    const std::size_t step = collection.front().value.size();
    std::set<int> classes;

    for (auto& f : collection) {
        classes.insert(f.classification);
        if (f.value.size() != step)
            throw std::runtime_error("Feature meassages does not match in size, cannot export!");
    }

    using ArffData = cslibs_arff::ArffData;
    using ArffAttribute = cslibs_arff::ArffAttr;
    using ArffInstance = cslibs_arff::ArffInstance;
    using ArffValue = cslibs_arff::ArffValue;

    ArffData arff;
    arff.set_relation_name(relation_name_);
    for (std::size_t i = 0; i < step; ++i) {
        arff.add_attr(std::make_shared<ArffAttribute>(ArffAttribute("feature_" + std::to_string(i), cslibs_arff::NUMERIC)));
    }

    arff.add_attr(std::make_shared<ArffAttribute>(ArffAttribute("class", cslibs_arff::NOMINAL)));
    for (int c : classes) {
        arff.add_nominal_val("class", std::to_string(c));
    }

    for (auto f : collection) {
        ArffInstance::Ptr i(new ArffInstance);
        for (float v : f.value) {
            i->add(std::make_shared<ArffValue>(v));
        }
        i->add(std::make_shared<ArffValue>(std::to_string(f.classification), false));
        arff.add_instance(i);
    }

    arff.write_arff(path_);

    return true;
}
