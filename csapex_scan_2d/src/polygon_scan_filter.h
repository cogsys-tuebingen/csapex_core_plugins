#ifndef POLYGONSCANFILTER_H
#define POLYGONSCANFILTER_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <utils_laser_processing/data/scan.h>
#include <csapex_scan_2d/labeled_scan_message.h>


namespace csapex
{

class Input;

class PolygonScanFilter : public InteractiveNode
{
    friend class PolygonScanFilterAdapter;

public:

    PolygonScanFilter();
    virtual ~PolygonScanFilter();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);

    void setResult(connection_types::LabeledScanMessage::Ptr result);

protected:
    virtual void beginProcess() override;
    virtual void finishProcess() override;

protected:
    Input* input_;
    Output* output_;

    connection_types::LabeledScanMessage::Ptr result_;

public:
    csapex::slim_signal::Signal<void(const lib_laser_processing::Scan* )> display_request;
    csapex::slim_signal::Signal<void()> reset_request;
};

}

#endif // POLYGONSCANFILTER_H
