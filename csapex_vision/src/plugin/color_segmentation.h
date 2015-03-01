#ifndef COLOR_SEGMENTATION_H
#define COLOR_SEGMENTATION_H

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex_vision/cv_mat_message.h>

namespace csapex
{

class ColorSegmentation : public csapex::Node
{
public:
    ColorSegmentation();

public:
    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

protected:
    virtual void setParameterState(Memento::Ptr memento);

private:
    void recompute();
    void update();

private:
    Input* input_img_;
    Input* input_mask_;

    Output* output_mask_;

    GenericStatePtr loaded_state_;

    cv::Scalar min, max;
    Encoding current_encoding;
};

}

#endif // COLOR_SEGMENTATION_H
