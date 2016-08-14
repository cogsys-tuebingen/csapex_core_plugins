#ifndef FILTER_SPLITTER_H
#define FILTER_SPLITTER_H

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex_vision/cv_mat_message.h>

namespace csapex {
class Splitter : public csapex::Node
{
    friend class SplitterSerializer;

public:
    Splitter();
    ~Splitter();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);

    virtual void process() override;

private:
    Input *input_;

    void updateOutputs();

    Encoding encoding_;
    int channel_count_;

};
}
#endif // FILTER_SPLITTER_H
