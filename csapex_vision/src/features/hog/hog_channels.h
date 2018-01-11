#ifndef HOG_CHANNELS_H
#define HOG_CHANNELS_H

/// PROJECT
#include <csapex/model/node.h>
#include "hog.h"

/// EXTRACT HOG FEATURE

namespace csapex {
class HOGChannels : public csapex::Node
{
public:
    HOGChannels();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    csapex::Input  *in_img_;
    csapex::Output *out_img_;

    int  bins_;
    bool signed_;

};
}
#endif // HOG_CHANNELS_H
