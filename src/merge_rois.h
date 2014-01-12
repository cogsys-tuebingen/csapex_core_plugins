#ifndef MERGE_ROIS_H
#define MERGE_ROIS_H


/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class MergeROIs : public csapex::Node
{
public:
    MergeROIs();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;
};

}

#endif // MERGE_ROIS_H
