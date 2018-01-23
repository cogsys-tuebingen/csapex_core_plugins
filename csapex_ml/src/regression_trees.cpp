/// HEADER
#include "regression_trees.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/signal/slot.h>
#include <csapex/model/node_handle.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::RegressionTrees, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

RegressionTrees::RegressionTrees() :
    loaded_(false)
{
}

void RegressionTrees::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("responses");

    reload_ = node_modifier.addSlot("Reload", [this](){loaded_ = false;});

}

void RegressionTrees::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declarePath("forest_path",
                                                                         csapex::param::ParameterDescription("Path to a saved svm."),
                                                                         true,
                                                                         ""),
                            path_);

}

void RegressionTrees::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr<std::vector<FeaturesMessage>> output(new std::vector<FeaturesMessage>);

    load();

    std::size_t size = input->size();
    for(std::size_t i = 0 ; i < size ; ++i)
    {
        cv::Mat sample(input->at(i).value);

        const FeaturesMessage& in_feature = input->at(i);
        FeaturesMessage out_feature;
        out_feature.frame_id = in_feature.frame_id;
        out_feature.stamp_micro_seconds = in_feature.stamp_micro_seconds;
        out_feature.value = in_feature.value;
        out_feature.type = FeaturesMessage::Type::REGRESSION;

        for(std::size_t j = 0 ; j < forests_.size() ; ++j) {

            RandomTreePtr tree = forests_.at(j);
#if CV_MAJOR_VERSION == 2
            float prediction_value =  tree->predict(sample);
#elif CV_MAJOR_VERSION == 3
            std::vector<float> results;
            float prediction_value = tree->predict(sample, results, cv::ml::StatModel::Flags::RAW_OUTPUT);
#endif
            out_feature.regression_result.emplace_back(prediction_value);
        }
        output->push_back(out_feature);
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_, output);
}



void RegressionTrees::load()
{
    if(loaded_)
        return;

    if(path_ == "")
        return;

    forests_.clear();

    const static std::string prefix = "reg_forest_";
    cv::FileStorage fs(path_, cv::FileStorage::READ);
    int num_forests = 0;
    fs["num_forests"] >> num_forests;
    cv::FileNode trees_node = fs["regression"];
    if(num_forests == 0) {
        throw std::runtime_error("File entry 'num_forests' has be defined!");
    }


    for(std::size_t i = 0 ; i < (std::size_t) num_forests ; ++i) {
        std::string label = prefix + std::to_string(i);

#if CV_MAJOR_VERSION == 2
        RandomTreePtr tree(new cv::RandomTrees);
        tree->read(fs.fs, (CvFileNode*) fs[label].node);
#elif CV_MAJOR_VERSION == 3
        RandomTreePtr tree = cv::ml::RTrees::create();
        cv::FileNode node = trees_node[label];
        tree->read(node);
#endif

        forests_.push_back(tree);


    }
    loaded_ = true;
}
