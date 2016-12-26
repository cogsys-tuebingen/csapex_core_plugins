#ifndef DECISION_TREE_FOREST_H
#define DECISION_TREE_FOREST_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN DecisionTreeForest : public Node
{
public:
    DecisionTreeForest();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    Input      *in_;
    Output     *out_;
    Slot       *reload_;
    bool        loaded_;

    cv::Mat     forest_responses_;
    std::size_t forest_size_;
#if CV_MAJOR_VERSION == 2
    typedef std::shared_ptr<cv::DecisionTree> DTreePtr;
    std::vector<DTreePtr> forest_;

#elif CV_MAJOR_VERSION == 3
    typedef cv::Ptr<cv::ml::DTrees> DTreePtr;
    std::vector<DTreePtr> forest_;
#endif


    void load();

};
}

#endif // DECISION_TREE_FOREST_H
