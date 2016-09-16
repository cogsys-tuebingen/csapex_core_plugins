/// HEADER
#include "random_trees.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/signal/slot.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::RandomTrees, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace impl {
inline void classify(const cv::RandomTrees        &random_trees,
                     const FeaturesMessage        &in_feature,
                     FeaturesMessage              &out_feature,
                     std::map<int, std::size_t>   &votes)
{
    out_feature = in_feature;

    const cv::Mat sample(1, in_feature.value.size(), CV_32FC1, out_feature.value.data());

    std::size_t max_votes    = 0;
    int max_class_id = 0;
    int ntrees = random_trees.get_tree_count();
    for(int i = 0 ; i < ntrees ; ++i) {
        CvDTreeNode* prediction = random_trees.get_tree(i)->predict(sample);
        int prediction_class_id = std::round(prediction->value);
        std::size_t &prediction_votes = votes[prediction_class_id];
        ++prediction_votes;

        if(prediction_votes > max_votes) {
            max_class_id = prediction_class_id;
            max_votes = prediction_votes;
        }
    }

    out_feature.confidence     = votes[max_class_id] / (float) ntrees;
    out_feature.classification = max_class_id;
}


}


RandomTrees::RandomTrees()
    : loaded_(false)
{
}

void RandomTrees::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareFileInputPath("path", "rforest.yaml"),
                            path_);
    parameters.addParameter(param::ParameterFactory::declareBool("compute_class_weights", false),
                            compute_class_weights_);
}


void RandomTrees::setup(NodeModifier& node_modifier)
{
    in_features_  = node_modifier.addInput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Unclassified feature");
    out_features_ = node_modifier.addOutput<GenericVectorMessage, csapex::connection_types::FeaturesMessage>("Classified feature");
    out_class_weights_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage::ConstPtr>("Class weights");

    reload_ = node_modifier.addSlot("Reload", std::bind(&RandomTrees::reloadTree, this));
}

void RandomTrees::process()
{
    std::shared_ptr<std::vector<FeaturesMessage> const> input_feature = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_features_);
    std::shared_ptr< std::vector<FeaturesMessage> > output_feature (new std::vector<FeaturesMessage>);
    std::shared_ptr<std::vector<CvMatMessage::ConstPtr>> output_class_weights;
    if(!loaded_) {
        if(path_ != "") {
            cv::FileStorage fs(path_, cv::FileStorage::READ);
            bool old_format = fs["random_forest"].node == nullptr;
            if (old_format)
            {
                fs.release();
                random_trees_.load(path_.c_str());
            }
            else
            {
                random_trees_.read(fs.fs, (CvFileNode*) fs["random_forest"].node);
                std::vector<int> class_labels;
                fs["classes"] >> class_labels;
                for(int c : class_labels) {
                    class_labels_[c] = 0;
                }
                fs.release();
            }
            loaded_ = true;
        } else {
            throw std::runtime_error("Randomforest couldn't be loaded!");
        }
    }
    std::size_t n = input_feature->size();
    output_feature->resize(n);

    if(compute_class_weights_) {
        output_class_weights.reset(new std::vector<CvMatMessage::ConstPtr>());
        for(std::size_t i = 0; i < n; ++i) {
            CvMatMessage::Ptr mat_message(new CvMatMessage(enc::unknown, 0));
            std::map<int, std::size_t> class_labels = class_labels_;
            impl::classify(random_trees_,
                           input_feature->at(i),
                           output_feature->at(i),
                           class_labels);
            cv::Mat &mat = mat_message->value;
            mat = cv::Mat(class_labels.size(), 2, CV_32FC1, cv::Scalar());
            int row = 0;
            for(const auto &entry : class_labels) {
                mat.at<float>(row, 0) = entry.first;
                mat.at<float>(row, 1) = entry.second / (float) random_trees_.get_tree_count();
            }
            output_class_weights->push_back(mat_message);
        }
    } else {
        for(std::size_t i = 0; i < n; ++i) {
            std::map<int, std::size_t> class_labels = class_labels_;
            impl::classify(random_trees_,
                           input_feature->at(i),
                           output_feature->at(i),
                           class_labels);
        }
    }

    msg::publish<GenericVectorMessage, FeaturesMessage>(out_features_, output_feature);
    if(output_class_weights)
        msg::publish<GenericVectorMessage, CvMatMessage::ConstPtr>(out_class_weights_, output_class_weights);
}

void RandomTrees::reloadTree()
{
    loaded_ = false;
}
