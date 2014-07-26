#ifndef TYPE_CONVERTER_H
#define TYPE_CONVERTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class ConvertType : public csapex::Node
{
public:
    ConvertType();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

private:
    csapex::Input  *input_;
    csapex::Output *output_;

    void update();

    int  mode_;
    bool normalize_;
};
}
#endif // TYPE_CONVERTER_H
