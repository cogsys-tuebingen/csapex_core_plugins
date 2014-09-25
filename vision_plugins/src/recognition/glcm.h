#ifndef GLCM_H
#define GLCM_H
/// PROJECT
#include <csapex/model/node.h>

namespace vision_plugins {
class GLCM : public csapex::Node
{
public:
    GLCM();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    csapex::Input  *in_;
    csapex::Output *out_;
};
}

#endif // GLCM_H
