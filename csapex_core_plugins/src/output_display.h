#ifndef OUTPUT_DISPLAY_H
#define OUTPUT_DISPLAY_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <QImage>
#include <QSharedPointer>

namespace csapex
{

class Input;

class OutputDisplay : public Node
{
    friend class OutputDisplayAdapter;

public:
    OutputDisplay();
    virtual ~OutputDisplay();

    void setup();
    void process();

protected:
    Input* input_;

public:
    boost::signals2::signal<void(QSharedPointer<QImage>)> display_request;
    MessageRendererPtr renderer_;
};

}

#endif // OUTPUT_DISPLAY_H
