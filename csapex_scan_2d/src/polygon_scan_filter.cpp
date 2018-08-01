/// SCAN
#include "polygon_scan_filter.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::PolygonScanFilter, csapex::Node)


using namespace csapex;
using namespace connection_types;
using namespace lib_laser_processing;

PolygonScanFilter::PolygonScanFilter()
{
}

PolygonScanFilter::~PolygonScanFilter()
{
}

void PolygonScanFilter::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareBool("invert",
                                                                 false),
                            invert_);
}

void PolygonScanFilter::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addMultiInput<ScanMessage, LabeledScanMessage>("Scan");
    output_ = node_modifier.addOutput<LabeledScanMessage>("Labeled Scan");
}

void PolygonScanFilter::beginProcess()
{
    result_.reset();

    if(msg::isMessage<LabeledScanMessage>(input_)) {
        LabeledScanMessage::ConstPtr scan_msg = msg::getMessage<LabeledScanMessage>(input_);
        display_request(&scan_msg->value, invert_);
    } else if(msg::isMessage<ScanMessage>(input_)) {
        ScanMessage::ConstPtr scan_msg = msg::getMessage<ScanMessage>(input_);
        display_request(&scan_msg->value, invert_);
    } else {
        throw std::runtime_error("invalid input type");
    }
}

void PolygonScanFilter::finishProcess()
{
    msg::publish(output_, result_);
}

void PolygonScanFilter::setResult(connection_types::LabeledScanMessage::Ptr result)
{
    result_ = result;
    done();
}
