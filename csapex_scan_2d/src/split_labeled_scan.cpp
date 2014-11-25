/// HEADER
#include "split_labeled_scan.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex_core_plugins/vector_message.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::SplitLabeledScan, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


SplitLabeledScan::SplitLabeledScan()
{
}

void SplitLabeledScan::setupParameters()
{
}

void SplitLabeledScan::setup()
{
    in_  = modifier_->addInput<LabeledScanMessage>("Labeled Scan");
    out_scan_ = modifier_->addOutput<ScanMessage>("Scan");
    out_labels_ = modifier_->addOutput<GenericVectorMessage, int>("Labels");
}

void SplitLabeledScan::process()
{
    LabeledScanMessage::Ptr input = in_->getMessage<LabeledScanMessage>();

    ScanMessage::Ptr output_scan(new ScanMessage);
    output_scan->value = input->value;


    out_scan_->publish(output_scan);

    boost::shared_ptr<std::vector<int> > labels(new std::vector<int>);
    *labels = input->value.labels;
    out_labels_->publish<GenericVectorMessage, int>(labels);
}

