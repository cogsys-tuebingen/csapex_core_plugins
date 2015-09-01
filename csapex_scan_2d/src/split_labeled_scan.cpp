/// HEADER
#include "split_labeled_scan.h"

/// PROJECT
#include <csapex/msg/io.h>
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

void SplitLabeledScan::setupParameters(Parameterizable& parameters)
{
}

void SplitLabeledScan::setup(NodeModifier& node_modifier)
{
    in_  = node_modifier.addInput<LabeledScanMessage>("Labeled Scan");
    out_scan_ = node_modifier.addOutput<ScanMessage>("Scan");
    out_labels_ = node_modifier.addOutput<GenericVectorMessage, int>("Labels");
}

void SplitLabeledScan::process()
{
    LabeledScanMessage::ConstPtr input = msg::getMessage<LabeledScanMessage>(in_);

    ScanMessage::Ptr output_scan(new ScanMessage);
    output_scan->value = input->value;


    msg::publish(out_scan_, output_scan);

    std::shared_ptr<std::vector<int> > labels(new std::vector<int>);
    *labels = input->value.labels;
    msg::publish<GenericVectorMessage, int>(out_labels_, labels);
}

