#ifndef SCAN_LABELER_H
#define SCAN_LABELER_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <cslibs_laser_processing/data/scan.h>

namespace csapex
{
class Input;

class ScanLabeler : public InteractiveNode
{
    friend class ScanLabelerAdapter;

public:
    ScanLabeler();
    virtual ~ScanLabeler();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

    void setResult(connection_types::LabeledScanMessage::Ptr result);

protected:
    void beginProcess() override;
    void finishProcess() override;

private:
    void submit();

protected:
    Input* input_;
    Output* output_;

    connection_types::LabeledScanMessage::Ptr result_;

public:
    slim_signal::Signal<void(const lib_laser_processing::Scan*)> display_request;
    slim_signal::Signal<void()> submit_request;

    connection_types::MessageConstPtr last_message_;
};

}  // namespace csapex

#endif  // SCAN_LABELER_H
