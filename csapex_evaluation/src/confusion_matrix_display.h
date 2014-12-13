#ifndef CONFUSION_MATRIX_H_
#define CONFUSION_MATRIX_H_

/// COMPONENT
#include <csapex_evaluation/confusion_matrix_message.h>

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class ConfusionMatrixDisplay : public Node
{
public:
    ConfusionMatrixDisplay();

    virtual void process();
    virtual void setup();

    const ConfusionMatrix& getConfusionMatrix() const;

public:
    boost::signals2::signal<void()> display_request;

private:
    Input* connector_;

    ConfusionMatrix confusion_;
};

}

#endif // CONFUSION_MATRIX_H_
