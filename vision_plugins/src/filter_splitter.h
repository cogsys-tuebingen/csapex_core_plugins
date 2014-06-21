#ifndef FILTER_SPLITTER_H
#define FILTER_SPLITTER_H

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex_vision/cv_mat_message.h>

namespace csapex {
class Splitter : public csapex::Node
{
    Q_OBJECT

public:
    Splitter();
    ~Splitter();

    void setup();

    void setState(Memento::Ptr memento);
    Memento::Ptr getChildState() const;

private Q_SLOTS:
    void process();

private:
    ConnectorIn *input_;

    void updateOutputs();

    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node);
        void writeYaml(YAML::Emitter &out) const;

    public:
        Encoding encoding_;
        int channel_count_;
    };

    State state_;

};
}
#endif // FILTER_SPLITTER_H
