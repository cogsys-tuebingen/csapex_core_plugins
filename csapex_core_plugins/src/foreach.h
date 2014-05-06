#ifndef FOREACH_H
#define FOREACH_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <QWaitCondition>

namespace csapex {

class Foreach : public csapex::Node
{
    Q_OBJECT

    friend class ForeachAdapter;

public:
    Foreach();
    ~Foreach();

    void checkIO();

    virtual void process();
    virtual void setup();

    virtual void stop();

private Q_SLOTS:
    void messageProcessed();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;

    ConnectorIn* in_sub;
    ConnectorOut* out_sub;

    bool msg_received_;
    QMutex msg_mutex_;
    QWaitCondition msg_received_cond_;
};

}

#endif // FOREACH_H
