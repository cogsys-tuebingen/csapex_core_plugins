#ifndef SVM_H
#define SVM_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <mutex>

namespace csapex {
class SVM : public Node
{
public:
    SVM();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    enum ThresholdType { GREATER = 0, LESS, LESS_EQUAL, GREATER_EQUAL};

    Input  *in_;
    Output *out_;
    Slot*   reload_;

    bool loaded_;
    cv::SVM svm_;

    void load();
};
}

#endif // SVM_H
