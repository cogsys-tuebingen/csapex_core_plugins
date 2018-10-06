/// HEADER
#include "decision_tree_forest.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::DecisionTreeForest, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

DecisionTreeForest::DecisionTreeForest() : loaded_(false)
{
}
void DecisionTreeForest::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("features");
    out_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage::ConstPtr>("responses");

    reload_ = node_modifier.addSlot("Reload", [this]() { load(); });
}

void DecisionTreeForest::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declarePath("forest_path", csapex::param::ParameterDescription("Path to a saved forest."), true, "", "*.yaml *.tar.gz"),
                            std::bind(&DecisionTreeForest::load, this));
}

void DecisionTreeForest::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
    std::shared_ptr<std::vector<CvMatMessage::ConstPtr>> output(new std::vector<CvMatMessage::ConstPtr>);

    if (!loaded_) {
        throw std::runtime_error("No forest is loaded!");
    }

    std::size_t size = input->size();
    for (std::size_t i = 0; i < size; ++i) {
        cv::Mat sample(input->at(i).value);
        CvMatMessage::Ptr result_msg(new CvMatMessage(enc::unknown, "unknown", 0));
        cv::Mat& result_value = result_msg->value;
        result_value = forest_responses_.clone();

        for (std::size_t j = 0; j < forest_size_; ++j) {
            DTreePtr& dtree = forest_.at(j);
#if CV_MAJOR_VERSION == 2
            result_value.at<float>(j, 1) = dtree->predict(sample)->class_idx;
#elif CV_MAJOR_VERSION == 3
            result_value.at<float>(j, 1) = dtree->predict(sample);
#endif
        }
        output->push_back(result_msg);
    }

    msg::publish<GenericVectorMessage, CvMatMessage::ConstPtr>(out_, output);
}

void DecisionTreeForest::load()
{
    std::string path = readParameter<std::string>("svm array path");
    if (path == "")
        return;

    forest_.clear();
    const std::string prefix = "dtree_";
    cv::FileStorage fs(path, cv::FileStorage::READ);
    std::vector<int> labels;
    fs["labels"] >> labels;
    if (labels.empty())
        throw std::runtime_error("File entry 'labels' may not be empty!");

    /// get svms for labels
    forest_responses_ = cv::Mat(labels.size(), 2, CV_32FC1, cv::Scalar());
    int size = labels.size();
    for (int i = 0; i < size; ++i) {
        std::string label = prefix + std::to_string(labels.at(i));
#if CV_MAJOR_VERSION == 2
        DTreePtr dtree(new cv::DecisionTree);
        dtree->read(fs.fs, (CvFileNode*)fs[label].node);
#elif CV_MAJOR_VERSION == 3
        DTreePtr dtree(cv::ml::StatModel::read<cv::ml::DTrees>(fs[label]));
#endif

        forest_.emplace_back(dtree);
        forest_responses_.at<float>(i, 0) = labels.at(i);
    }

    forest_size_ = forest_.size();

    loaded_ = true;
}
