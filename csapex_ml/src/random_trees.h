#ifndef RANDOM_TREES_H
#define RANDOM_TREES_H

/// COMPONENT
#include <csapex_ml/features_message.h>
#include <csapex_opencv/cv_mat_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{
#if CV_MAJOR_VERSION == 2
typedef cv::RandomTrees RandomTree;
#elif CV_MAJOR_VERSION == 3
typedef cv::Ptr<cv::ml::RTrees> RandomTree;
#endif

class CSAPEX_EXPORT_PLUGIN RandomTrees : public csapex::Node
{
public:
    RandomTrees();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    void reloadTree();

private:
    Input* in_features_;
    Output* out_features_;
    Output* out_class_weights_;

    Slot* reload_;

    RandomTree random_trees_;
    std::string path_;
    bool loaded_;
    bool compute_class_weights_;
    std::map<int, std::size_t> class_labels_;
};

}  // namespace csapex

#endif  // RANDOM_TREES_H
