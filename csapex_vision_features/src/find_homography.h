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

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

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

}  // namespace csapex

#endif  // FIND_HOMOGRAPHY_H
