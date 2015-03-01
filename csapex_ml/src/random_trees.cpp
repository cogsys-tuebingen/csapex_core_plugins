/// HEADER
#include "random_trees.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/signal/slot.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::RandomTrees, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


RandomTrees::RandomTrees()
    : loaded_(false)
{
}

void RandomTrees::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareFileInputPath("file", "rforest.yaml"), std::bind(&RandomTrees::loadTree, this));
}

void RandomTrees::setup(NodeModifier& node_modifier)
{
    in_  = node_modifier.addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Unclassified feature");
    out_ = node_modifier.addOutput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Classified feature");

    reload_ = node_modifier.addSlot("Reload", std::bind(&RandomTrees::loadTree, this));
}

void RandomTrees::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr< std::vector<FeaturesMessage> > output (new std::vector<FeaturesMessage>);

    std::size_t n = input->size();

    if(loaded_) {
        output->resize(n);
        for(std::size_t i = 0; i < n; ++i) {
            output->at(i) = classify(input->at(i));
        }

    } else {
        *output = *input;
        modifier_->setWarning("cannot classfiy, no forest loaded");
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}

connection_types::FeaturesMessage RandomTrees::classify(const FeaturesMessage &input)
{
    connection_types::FeaturesMessage result = input;

    cv::Mat feature(1, input.value.size(), CV_32FC1, result.value.data());
    float prediction = dtree_.predict(feature);

    result.classification = std::round(prediction);
    return result;
}

void RandomTrees::loadTree()
{
    std::string file = readParameter<std::string>("file");
    dtree_.load(file.c_str());

    loaded_ = dtree_.get_tree_count() > 0;
}
