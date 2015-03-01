#ifndef SCALE_H
#define SCALE_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/core/core.hpp>

namespace vision_plugins {
class Scale : public csapex::Node
{
public:
    Scale();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

private:
    csapex::Output*   output_;
    csapex::Input*    input_;

    cv::Vec2d               scales_;
    int                     mode_;
    void update();

};

}
#endif // SCALE_H
