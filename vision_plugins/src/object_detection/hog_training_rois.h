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

    void setupParameters();
    void setup();
    void process();

private:
    ConnectorIn*    in_image_;
    ConnectorIn*    in_roi_;
    ConnectorOut*   out_;
};


}

#endif // HOG_TRAINING_ROIS_H
