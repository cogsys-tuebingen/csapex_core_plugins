#ifndef SPLIT_LABELED_SCAN_H
#define SPLIT_LABELED_SCAN_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class SplitLabeledScan : public csapex::Node
{
public:
    SplitLabeledScan();

    void setupParameters();
    void setup();
    void process();

private:
    Input* in_;
    Output* out_scan_;
    Output* out_labels_;
};


}

#endif // SPLIT_LABELED_SCAN_H
