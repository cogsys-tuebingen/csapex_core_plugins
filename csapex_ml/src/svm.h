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
    Input  *in_;
    Output *out_;

    std::mutex m_;
    bool         loaded_;
    cv::SVM      svm_;

    void load();
};
}

#endif // SVM_H
