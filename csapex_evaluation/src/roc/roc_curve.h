#ifndef ROC_CURVE_H
#define ROC_CURVE_H

/// COMPONENT
#include <csapex_evaluation/confusion_matrix_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <mutex>

namespace csapex {

class ROCCurve : public Node
{
public:
    struct Entry {
        double threshold;
        double tpr;
        double fpr;
    };

public:
    ROCCurve();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

    const std::map<double, Entry>& getEntries() const;

public:
    csapex::slim_signal::Signal<void()> display_request;

private:
    Input* in_confusion_;
    Input* in_threshold_;

    mutable std::recursive_mutex mutex_;


    std::map<double, Entry> entries_;
};

}

#endif // ROC_CURVE_H
