/// HEADER
#include "labeled_scan_to_scan.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::LabeledScanToScan, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

LabeledScanToScan::LabeledScanToScan()
{
}

void LabeledScanToScan::setup()
{
    in_ = modifier_->addInput<LabeledScanMessage>("Labeled Scan");
    out_ = modifier_->addOutput<ScanMessage>("Scan");
}

void LabeledScanToScan::process()
{
    LabeledScanMessage::Ptr lmsg = in_->getMessage<LabeledScanMessage>();

    ScanMessage::Ptr msg(new ScanMessage);
    msg->value = lmsg->value;

    out_->publish(msg);
}
