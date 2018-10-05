#ifndef REGRESSION_TREES_H
#define REGRESSION_TREES_H

/// COMPONENT
#include <csapex/param/range_parameter.h>
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>
#include <opencv2/opencv.hpp>
namespace csapex
{
#if CV_MAJOR_VERSION == 2
typedef std::shared_ptr<cv::RandomTrees> RandomTreePtr;
#elif CV_MAJOR_VERSION == 3
typedef cv::Ptr<cv::ml::RTrees> RandomTreePtr;
#endif

class CSAPEX_EXPORT_PLUGIN RegressionTrees : public Node
{
public:
    RegressionTrees();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    Input* in_;
    Output* out_;
    Slot* reload_;

    bool loaded_;
    std::vector<RandomTreePtr> forests_;
    std::string path_;
    bool compute_class_weights_;

    void load();
    void updateThresholds();
};
}  // namespace csapex

#endif  // REGRESSION_TREES_H
