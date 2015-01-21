#ifndef TO_FEATURE_H
#define TO_FEATURE_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>
#include <csapex_core_plugins/vector_message.h>

namespace csapex {

class EmptyLabeledFeaturesMessage : public Node
{
public:
    EmptyLabeledFeaturesMessage();

    virtual void setup();
    virtual void setupParameters();
    virtual void process();
    virtual void tick();

protected:
    csapex::Output  *output_;


};

}
#endif // TO_FEATURE_H
