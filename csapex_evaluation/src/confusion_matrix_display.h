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
    virtual void setupParameters(csapex::Parameterizable& params) override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

    const ConfusionMatrix& getConfusionMatrix() const;
    void save();

    void exportCsv(const std::string& file);

public:
    slim_signal::Signal<void()> display_request;
    slim_signal::Signal<void()> export_request;


private:
    Input* connector_;

    mutable std::recursive_mutex mutex_;
    ConfusionMatrix confusion_;
    std::string filename_;
    std::string path_;
};

}

#endif // CONFUSION_MATRIX_DISPLAY_H
