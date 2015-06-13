#ifndef FIND_HOMOGRAPHY_H
#define FIND_HOMOGRAPHY_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class FindHomography : public csapex::Node
{
private:

public:
    FindHomography();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    void update();

private:
    Input* in_img_1;
    Input* in_key_1;
    Input* in_img_2;
    Input* in_key_2;
    Input* in_matches;

    Output* out_img;

    int method_;
    double ransac_threshold_;
};

} /// NAMESPACE

#endif // FIND_HOMOGRAPHY_H
