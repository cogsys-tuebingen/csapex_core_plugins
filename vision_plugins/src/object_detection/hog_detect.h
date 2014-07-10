#ifndef HOG_DETECT_H
#define HOG_DETECT_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace vision_plugins {


class HOGDetect : public csapex::Node
{
public:
    HOGDetect();
    virtual ~HOGDetect();

    void setupParameters();
    void setup();
    void process();

private:
    enum SVMType        {DEFAULT, DAIMLER, CUSTOM};
    enum DetectionType  {SINGLE_SCALE, MULTI_SCALE};

    csapex::ConnectorIn*  in_;
    csapex::ConnectorOut* out_;

    std::vector<float>    svm_;

    void load();
};


}

#endif // HOG_DETECT_H
