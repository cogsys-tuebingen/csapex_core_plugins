/// HEADER
#include "labeled_scan_to_scan.h"

/// PROJECT
#include <csapex/msg/io.h>
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

void LabeledScanToScan::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<LabeledScanMessage>("Labeled Scan");
    out_ = node_modifier.addOutput<ScanMessage>("Scan");
}

void LabeledScanToScan::process()
{
    LabeledScanMessage::ConstPtr lmsg = msg::getMessage<LabeledScanMessage>(in_);

    ScanMessage::Ptr msg(new ScanMessage);
    msg->value = lmsg->value;

    msg::publish(out_, msg);
}
