#ifndef FILTER_SPLITTER_H
#define FILTER_SPLITTER_H

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex_vision/cv_mat_message.h>

namespace csapex {
class Splitter : public csapex::Node
{
public:
    Splitter();
    ~Splitter();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);

    virtual void process() override;

private:
    Input *input_;

    void updateOutputs();

    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node);
        void writeYaml(YAML::Node &out) const;

    public:
        Encoding encoding_;
        int channel_count_;
    };

    State state_;

};
}
#endif // FILTER_SPLITTER_H
