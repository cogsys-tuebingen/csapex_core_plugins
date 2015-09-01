#ifndef IMAGE_COLLAGE_H
#define IMAGE_COLLAGE_H

/// PROJECT
#include <csapex/model/node.h>
#include <utils_param/parameter.h>
#include <utils_param/range_parameter.h>

namespace csapex {


class ImageCollage : public csapex::Node
{
public:
    ImageCollage();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* in_main_;
    Input* in_secondary_;
    Output* out_;

    param::RangeParameter::Ptr p_x_;
    param::RangeParameter::Ptr p_y_;
};


}

#endif // IMAGE_COLLAGE_H
