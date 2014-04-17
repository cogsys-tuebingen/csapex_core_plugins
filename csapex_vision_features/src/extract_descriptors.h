#ifndef EXTRACT_DESCRIPTORS_H
#define EXTRACT_DESCRIPTORS_H

/// COMPONENT
#include <csapex/model/node.h>

/// PROJECT
#include <utils/extractor.h>

/// SYSTEM
#include <QCheckBox>
#include <QComboBox>
#include <QMutex>

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
    QMutex extractor_mutex;
    Extractor::Ptr extractor;

    ConnectorIn* in_img;
    ConnectorIn* in_key;
    ConnectorOut* out_des;
};

}

#endif // EXTRACT_DESCRIPTORS_H
