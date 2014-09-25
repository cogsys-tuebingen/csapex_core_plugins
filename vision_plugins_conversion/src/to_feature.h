#ifndef TO_FEATURE_H
#define TO_FEATURE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>
#include <csapex_core_plugins/vector_message.h>

namespace vision_plugins {

class ToFeature : public csapex::Node
{
public:
    ToFeature();

    virtual void setup();
    virtual void setupParameters();
    virtual void process();

protected:
    csapex::Input   *input_;
    csapex::Output  *output_;


};

}
#endif // TO_FEATURE_H
