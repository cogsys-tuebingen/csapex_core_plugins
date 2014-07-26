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
    Input* in_;
    Output* out_;
};


}

#endif // HOG_TRAINING_ROIS_H
