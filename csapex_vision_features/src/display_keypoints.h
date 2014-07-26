#ifndef DisplayFeatures_H
#define DisplayFeatures_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{

class DisplayKeypoints : public csapex::Node
{
public:
    DisplayKeypoints();

public:
    virtual void setup();

    virtual void process();

private:
    Input* in_img;
    Input* in_key;

    Output* out_img;
};

}

#endif // DisplayFeatures_H
