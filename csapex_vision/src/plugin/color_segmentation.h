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
    virtual void process();
    virtual void setup();

protected:
    virtual void setState(Memento::Ptr memento);

private:
    void recompute();
    void update();

private:
    ConnectorIn* input_img_;
    ConnectorIn* input_mask_;

    ConnectorOut* output_mask_;

    GenericStatePtr loaded_state_;

    cv::Scalar min, max;
    Encoding current_encoding;
};

}

#endif // COLOR_SEGMENTATION_H
