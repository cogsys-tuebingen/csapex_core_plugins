#ifndef POLYGONSCANFILTER_H
#define POLYGONSCANFILTER_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <cslibs_laser_processing/data/scan.h>

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
    void setupParameters(Parameterizable& parameters) override;

    void setResult(connection_types::LabeledScanMessage::Ptr result);

protected:
    void beginProcess() override;
    void finishProcess() override;

protected:
    Input* input_;
    Output* output_;
    bool invert_;

    connection_types::LabeledScanMessage::Ptr result_;

public:
    slim_signal::Signal<void(const lib_laser_processing::Scan*, const bool)> display_request;
};

}  // namespace csapex

#endif  // POLYGONSCANFILTER_H
