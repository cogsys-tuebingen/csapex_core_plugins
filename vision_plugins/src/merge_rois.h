#ifndef MERGE_ROIS_H
#define MERGE_ROIS_H


/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class MergeROIs : public csapex::Node
{
public:
    MergeROIs();

    virtual void process();
    virtual void setup();

private:
    Input* input_;
    Output* output_;
};

}

#endif // MERGE_ROIS_H
