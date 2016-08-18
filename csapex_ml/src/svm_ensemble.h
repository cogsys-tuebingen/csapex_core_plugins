#ifndef SVM_ARRAY_H
#define SVM_ARRAY_H

/// COMPONENT
#include <csapex_ml/features_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <mutex>

namespace csapex {
class CSAPEX_EXPORT_PLUGIN SVMEnsemble : public Node
{
public:
    SVMEnsemble();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    typedef std::shared_ptr<cv::SVM> SVMPtr;

    enum ThresholdType { GREATER = 0, LESS, LESS_EQUAL, GREATER_EQUAL};
    enum ClassTypes {NEGATIVE = -1, POSITIVE = 1};

    Input  *in_;
    Output *out_;
    Slot*   reload_;

    bool loaded_;
    std::vector<SVMPtr> svms_;
    std::size_t         svms_size_;
    cv::Mat             svm_responses_;

    void load();
};
}

#endif // SVM_ARRAY_H
