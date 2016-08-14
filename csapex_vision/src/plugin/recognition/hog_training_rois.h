#ifndef HOG_TRAINING_ROIS_H
#define HOG_TRAINING_ROIS_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class HOGTrainingRois : public csapex::Node
{
public:
    HOGTrainingRois();

    void setupParameters(Parameterizable& parameters);
    void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input*    in_image_;
    Input*    in_roi_;
    Output*   out_;

};


}

#endif // HOG_TRAINING_ROIS_H
