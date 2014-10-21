#ifndef EXTRACT_FEATURES_H
#define EXTRACT_FEATURES_H

/// COMPONENT
#include <csapex/model/node.h>

/// PROJECT
#include <utils_vision/utils/extractor.h>

namespace csapex
{

class ExtractKeypoints : public csapex::Node
{
public:
    ExtractKeypoints();

public:
    void setup();
    virtual void process();

private:
    void update();

private:
    Extractor::Ptr extractor;

    Input* in_img;
    Input* in_mask;
    Output* out_key;

    bool refresh_;
};

}

#endif // EXTRACT_FEATURES_H
