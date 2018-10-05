#ifndef SVM_ARRAY_H
#define SVM_ARRAY_H

/// COMPONENT
#include <csapex/param/range_parameter.h>
#include <csapex_ml/features_message.h>

/// PROJECT
#include "extended_svm.hpp"
#include <csapex/model/node.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN SVMEnsemble : public Node
{
public:
    SVMEnsemble();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
#if CV_MAJOR_VERSION == 2
    typedef std::shared_ptr<cv::SVM> SVMPtr;
#elif CV_MAJOR_VERSION == 3
    typedef cv::Ptr<cv::ml::SVM> SVMPtr;
#endif

    enum ThresholdType
    {
        GREATER = 0,
        LESS,
        LESS_EQUAL,
        GREATER_EQUAL
    };
    enum ClassTypes
    {
        NEGATIVE = -1,
        POSITIVE = 1
    };

    Input* in_;
    Output* out_;
    Slot* reload_;

    bool loaded_;
#if CV_MAJOR_VERSION == 2
    std::vector<ExtendedSVM::Ptr> svms_;
#elif CV_MAJOR_VERSION == 3
    std::vector<SVMPtr> svms_;
#endif
    std::size_t svms_size_;
    cv::Mat svm_responses_;
    std::vector<param::RangeParameter::Ptr> params_thresholds_;
    std::vector<double> thresholds_;

    void load();
    void updateThresholds();
};
}  // namespace csapex

#endif  // SVM_ARRAY_H
