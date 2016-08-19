#ifndef SVM_ARRAY_H
#define SVM_ARRAY_H

/// COMPONENT
#include <csapex_ml/features_message.h>
#include <csapex/param/range_parameter.h>

/// PROJECT
#include <csapex/model/node.h>
#include "extended_svm.hpp"

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
    Slot   *reload_;

    bool                                    loaded_;
    std::vector<ExtendedSVM::Ptr>           svms_;
    std::size_t                             svms_size_;
    cv::Mat                                 svm_responses_;
    std::vector<param::RangeParameter::Ptr> params_thresholds_;
    std::vector<double>                     thresholds_;

    void                                    load();
    void                                    updateThresholds();
};
}

#endif // SVM_ARRAY_H
