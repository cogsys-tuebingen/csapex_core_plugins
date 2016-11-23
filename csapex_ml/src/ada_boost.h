#ifndef ADABOOST_H
#define ADABOOST_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN AdaBoost : public Node
{
public:
    AdaBoost();

    virtual void setup(NodeModifier &node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    Input  *in_;
    Output *out_;
    Slot   *reload_;

    bool        loaded_;
    bool        compute_labels_;

#if CV_MAJOR_VERSION == 2
    cv::Boost   boost_;
#elif CV_MAJOR_VERSION == 3
    cv::Ptr<cv::ml::Boost>   boost_;
#endif
    std::string path_;

    void reload();
    void updateMethod();
};
}
#endif // ADABOOST_H
