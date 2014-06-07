#ifndef SCAN_LABELER_H
#define SCAN_LABELER_H

/// PROJECT
#include <csapex_core_plugins/interactive_node.h>
#include <utils_laser_processing/data/scan.h>
#include <csapex_scan_2d/labeled_scan_message.h>

namespace csapex
{

class ConnectorIn;

class ScanLabeler : public InteractiveNode
{
    friend class ScanLabelerAdapter;

public:
    ScanLabeler();
    virtual ~ScanLabeler();

    virtual QIcon getIcon() const;

    void setup();
    void setupParameters();
    void process();

    void setResult(connection_types::LabeledScanMessage::Ptr result);

private:
    void submit();

protected:
    ConnectorIn* input_;
    ConnectorOut* output_;

    connection_types::LabeledScanMessage::Ptr result_;

public:
    boost::signals2::signal<void(lib_laser_processing::Scan* )> display_request;
    boost::signals2::signal<void()> submit_request;
};

}

#endif // SCAN_LABELER_H
