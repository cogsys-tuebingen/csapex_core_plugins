#ifndef CONFUSION_MATRIX_DISPLAY_H
#define CONFUSION_MATRIX_DISPLAY_H

/// COMPONENT
#include <csapex_evaluation/confusion_matrix_message.h>

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <mutex>

namespace csapex {

class ConfusionMatrixDisplay : public Node
{
public:
    ConfusionMatrixDisplay();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

    const ConfusionMatrix& getConfusionMatrix() const;

public:
    csapex::slim_signal::Signal<void()> display_request;

private:
    Input* connector_;

    mutable std::recursive_mutex mutex_;
    ConfusionMatrix confusion_;
};

}

#endif // CONFUSION_MATRIX_DISPLAY_H
