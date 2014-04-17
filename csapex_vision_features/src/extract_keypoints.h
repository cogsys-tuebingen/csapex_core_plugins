#ifndef EXTRACT_FEATURES_H
#define EXTRACT_FEATURES_H

/// COMPONENT
#include <csapex/model/node.h>

/// PROJECT
#include <config/reconfigurable.h>
#include <utils/extractor.h>

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

    ConnectorIn* in_img;
    ConnectorIn* in_mask;
    ConnectorOut* out_key;
};

}

#endif // EXTRACT_FEATURES_H
