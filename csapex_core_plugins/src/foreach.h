#ifndef FOREACH_H
#define FOREACH_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_core_plugins/vector_message.h>

namespace csapex {

class Foreach : public csapex::Node
{
    Q_OBJECT

    friend class ForeachAdapter;

public:
    Foreach();
    ~Foreach();

    virtual void process();
    virtual void setup();

    virtual void stop();

private Q_SLOTS:
    void appendMessageFrom(Connectable*);

private:
    ConnectorIn* input_;
    ConnectorOut* output_;

    ConnectorIn* in_sub;
    ConnectorOut* out_sub;

    connection_types::VectorMessage::Ptr current_result_;

    int messages_;
    int message_;
};

}

#endif // FOREACH_H
