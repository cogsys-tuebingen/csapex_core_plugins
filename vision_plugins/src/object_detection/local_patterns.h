#ifndef LOCAL_PATTERNS_H
#define LOCAL_PATTERNS_H

/// PROJECT
#include <csapex/model/node.h>
namespace vision_plugins {
class LocalPatterns : public csapex::Node
{
public:
    LocalPatterns();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    enum Type {LBP, LTP};

    csapex::Input  *in_img_;
    csapex::Input  *in_rois_;
    csapex::Output *out_;
};
}
#endif // LOCAL_PATTERNS_H
