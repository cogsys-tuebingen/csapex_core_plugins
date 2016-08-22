/// HEADER
#include "waldboost_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>

CSAPEX_REGISTER_CLASS(csapex::WaldBoostTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

WaldBoostTrainer::WaldBoostTrainer()
{
}

void WaldBoostTrainer::setup(NodeModifier &modifier)
{
    CollectionNode<connection_types::FeaturesMessage>::setup(modifier);
}

void WaldBoostTrainer::setupParameters(Parameterizable &parameters)
{
    CollectionNode<connection_types::FeaturesMessage>::setupParameters(parameters);
    parameters.addParameter(param::ParameterFactory::declareFileOutputPath("/waldboost/path",
                                                                           param::ParameterDescription("File to write boosted classifier to."),
                                                                           "",
                                                                           "*.yaml *.tar.gz"),
                 path_);
    parameters.addParameter(param::ParameterFactory::declareRange("/waldboost/classifier_count",
                                                                  1,
                                                                  4096,
                                                                  100,
                                                                  1),
                            weak_count_);
}

bool WaldBoostTrainer::processCollection(std::vector<FeaturesMessage> &collection)
{
    std::size_t step = collection.front().value.size();
    std::map<int, std::vector<std::size_t>> indices;

    const std::size_t size = collection.size();
    for(std::size_t i = 0 ; i < size ; ++i) {
        const FeaturesMessage &fm = collection[i];
        if(fm.value.size() != step)
            throw std::runtime_error("All descriptors must have the same length!");
        if(fm.classification != 1 &&
                fm.classification != -1)
            throw std::runtime_error("Only class labels supported are '-1' and '1'!");
        indices[fm.classification].push_back(i);
    }

    const std::vector<std::size_t> &positive_indices = indices[1];
    const std::vector<std::size_t> &negative_indices = indices[-1];

    const std::size_t positive_size = positive_indices.size();
    const std::size_t negative_size = negative_indices.size();

    cv::Mat positive_samples(positive_size, step, CV_32FC1, cv::Scalar());
    cv::Mat negative_samples(negative_size, step, CV_32FC1, cv::Scalar());

    for(std::size_t i = 0 ; i < positive_size ; ++i) {
        const std::vector<float> &sample = collection[positive_indices[i]].value;
        for(std::size_t j = 0 ; j < step ; ++j) {
            positive_samples.at<float>(i,j) = sample[j];
        }
    }

    for(std::size_t i = 0 ; i < negative_size ; ++i) {
        const std::vector<float> &sample = collection[negative_indices[i]].value;
        for(std::size_t j = 0 ; j < step ; ++j) {
            negative_samples.at<float>(i,j) = sample[j];
        }
    }

    cv::transpose(positive_samples, positive_samples);
    cv::transpose(negative_samples, negative_samples);

    cv::WaldBoost wb;
    wb.reset(weak_count_);
    std::cout << "[WaldBoost]: Started training with " << collection.size() << " samples!" << std::endl;
    wb.train(positive_samples, negative_samples);
    wb.save(path_);
    std::cout << "[WaldBoost]: Finished training!" << std::endl;

    return true;
}
