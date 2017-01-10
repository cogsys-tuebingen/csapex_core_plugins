#include <opencv2/opencv.hpp>

#if CV_MAJOR_VERSION == 2
/// HEADER
#include "gradient_boosted_trees.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/signal/slot.h>

CSAPEX_REGISTER_CLASS(csapex::GradientBoostedTrees, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

void GradientBoostedTrees::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareFileInputPath("path", "gbtree.yaml", "*.yaml"),
                            std::bind(&GradientBoostedTrees::load, this));
}

void GradientBoostedTrees::setup(NodeModifier &node_modifier)
{
    in_  = node_modifier.addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Unclassified feature");
    out_ = node_modifier.addOutput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Classified feature");

    reload_ = node_modifier.addSlot("Reload", [this](){load();});
}

void GradientBoostedTrees::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr< std::vector<FeaturesMessage> > output (new std::vector<FeaturesMessage>);

    std::size_t n = input->size();
    if(trees_) {
        output->resize(n);
        for(std::size_t i = 0 ; i < n ; ++i) {
            classify(input->at(i), output->at(i));
        }
    } else {
        throw std::runtime_error("No gradient boosted trees are loaded!");
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}

void GradientBoostedTrees::load()
{
    const std::string path = readParameter<std::string>("path");
    trees_.reset();

    std::shared_ptr<cv::GradientBoostingTrees> trees(new cv::GradientBoostingTrees);
    trees->load(path.c_str());
    trees_ = trees;
}

void GradientBoostedTrees::classify(const FeaturesMessage &input, FeaturesMessage &output)
{
    FeaturesMessage result = input;
    const cv::Mat feature(1, input.value.size(), CV_32FC1, result.value.data());
    result.classification = trees_->predict(feature);
}

#else
#warning Gradient boosted trees are not supported in OpenCV 3.
#endif
