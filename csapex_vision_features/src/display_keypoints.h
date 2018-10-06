#ifndef DisplayFeatures_H
#define DisplayFeatures_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{
class DisplayKeypoints : public csapex::Node
{
public:
    DisplayKeypoints();

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

    virtual void process() override;

private:
    Input* in_img;
    Input* in_key;

    Output* out_img;
};

}  // namespace csapex

#endif  // DisplayFeatures_H
