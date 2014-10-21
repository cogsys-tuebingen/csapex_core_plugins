#ifndef EXTRACT_DESCRIPTORS_H
#define EXTRACT_DESCRIPTORS_H

/// COMPONENT
#include <csapex/model/node.h>

/// PROJECT
#include <utils_vision/utils/extractor.h>

namespace csapex
{

class ExtractDescriptors : public csapex::Node
{
public:
    ExtractDescriptors();    

public:
    void setup();
    virtual void process();

private:
    void update();

private:
    Extractor::Ptr extractor;

    Input* in_img;
    Input* in_key;
    Output* out_des;

    bool refresh_;
};

}

#endif // EXTRACT_DESCRIPTORS_H
