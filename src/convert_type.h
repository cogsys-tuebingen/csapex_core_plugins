#ifndef TYPE_CONVERTER_H
#define TYPE_CONVERTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class ConvertType : public csapex::Node
{
public:
    ConvertType();

    virtual void process();
    virtual void setup();

private:
    ConnectorIn  *input_;
    ConnectorOut *output_;

    void update();

    int  mode_;
    bool normalize_;
};
}
#endif // TYPE_CONVERTER_H
