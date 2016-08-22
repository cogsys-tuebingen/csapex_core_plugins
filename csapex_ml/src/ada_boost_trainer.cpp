/// HEADER
#include "ada_boost_trainer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>


CSAPEX_REGISTER_CLASS(csapex::AdaBoostTrainer, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

AdaBoostTrainer::AdaBoostTrainer()
{

}

void AdaBoostTrainer::setupParameters(Parameterizable &parameters)
{
    CollectionNode<connection_types::FeaturesMessage>::setupParameters(parameters);
    parameters.addParameter(param::ParameterFactory::declareFileOutputPath("boost/path",
                                                                           "",
                                                                           "*.yaml, *.tar.gz"),
                            path_);
    std::map<std::string, int> split_criteria = {
        {"DEFAULT", cv::Boost::DEFAULT},
        {"GINI", cv::Boost::GINI},
        {"MISCLASS", cv::Boost::MISCLASS},
        {"SQERR", cv::Boost::SQERR}
    };

    parameters.addParameter(param::ParameterFactory::declareParameterSet("boost/split_criteria",
                                                                         split_criteria,
                                                                         (int) cv::Boost::DEFAULT),
                            boost_params_.split_criteria);

    std::map<std::string, int> boost_types = {
        {"DISCRETE", cv::Boost::DISCRETE},
        {"REAL", cv::Boost::REAL},
        {"LOGIT", cv::Boost::LOGIT},
        {"GENTLE", cv::Boost::GENTLE}
    };

    parameters.addParameter(param::ParameterFactory::declareParameterSet("boost/type",
                                                                         boost_types,
                                                                         (int) cv::Boost::DISCRETE),
                            boost_params_.boost_type);

    parameters.addParameter(param::ParameterFactory::declareRange("boost/classifier_count",
                                                                  1,
                                                                  4096,
                                                                  100,
                                                                  1),
                            boost_params_.weak_count);

    parameters.addParameter(param::ParameterFactory::declareRange("boost/trim_rate",
                                                                  0.0,
                                                                  1.0,
                                                                  0.0,
                                                                  0.01),
                            weight_trim_rate_);

    parameters.addParameter(param::ParameterFactory::declareRange("boost/max_depth",
                                                                  1,
                                                                  64,
                                                                  8,
                                                                  1),
                            boost_params_.max_depth);

    parameters.addParameter(param::ParameterFactory::declareBool("boost/use_surrogates",
                                                                 false),
                            boost_params_.use_surrogates);
}

bool AdaBoostTrainer::processCollection(std::vector<FeaturesMessage> &collection)
{
    std::size_t step = collection.front().value.size();
    for(const FeaturesMessage &fm : collection) {
        if(fm.value.size() != step)
            throw std::runtime_error("All descriptors must have the same length!");
        if(fm.classification != 1 &&
                fm.classification != -1)
            throw std::runtime_error("Only class labels supported are '-1' and '1'!");
    }

    boost_params_.weight_trim_rate = weight_trim_rate_;
    cv::Boost boost;
    cv::Mat samples(collection.size(), step, CV_32FC1, cv::Scalar());
    cv::Mat labels(collection.size(), 1, CV_32SC1, cv::Scalar());
    cv::Mat missing(collection.size(), step, CV_8UC1, cv::Scalar());
    for(int i = 0 ; i < samples.rows ; ++i) {
        labels.at<int>(i) = collection.at(i).classification;
        for(int j = 0 ; j < samples.cols ; ++j) {
            const float value = collection.at(i).value.at(j);
            if(std::isnan(value) ||
                    std::isinf(value) ||
                        (fabs(value) >= 0.5f * FLT_MAX)) {
                missing.at<uchar>(i,j) = 1;
                samples.at<float>(i,j) = 0.f;
            } else {
                samples.at<float>(i,j) = value;
            }
        }
    }

    std::cout << "[AdaBoost]: Started training with " << samples.rows << " samples!" << std::endl;
    if(boost.train(samples, CV_ROW_SAMPLE, labels, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), boost_params_)) {
        std::cout << "[AdaBoost]: Finished training" << std::endl;
        boost.save(path_.c_str(), "adaboost");
    } else {
        return false;
    }
    return true;
}
