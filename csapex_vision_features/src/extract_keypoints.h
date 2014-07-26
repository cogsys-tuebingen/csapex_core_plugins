#ifndef EXTRACT_FEATURES_H
#define EXTRACT_FEATURES_H

/// COMPONENT
#include <csapex/model/node.h>

/// PROJECT
#include <utils_vision/utils/extractor.h>

/// SYSTEM
#include <QCheckBox>
#include <QComboBox>
#include <QMutex>

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
    QMutex extractor_mutex;
    Extractor::Ptr extractor;

    Input* in_img;
    Input* in_mask;
    Output* out_key;
};

}

#endif // EXTRACT_FEATURES_H
