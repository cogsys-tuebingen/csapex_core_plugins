#ifndef SEGMENT_LABELER_H
#define SEGMENT_LABELER_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class SegmentLabeler : public csapex::Node
{
public:
    SegmentLabeler();

    void setupParameters();
    void setup();
    void process();

private:
    Input* in_segments_;
    Input* in_labeled_scan_;
    Output* out_;
};


}

#endif // SEGMENT_LABELER_H
