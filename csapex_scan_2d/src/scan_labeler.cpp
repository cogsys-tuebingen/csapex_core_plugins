/// HEADER
#include "scan_labeler.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_qt/QtCvImageConverter.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::ScanLabeler, csapex::Node)


using namespace csapex;
using namespace connection_types;
using namespace lib_laser_processing;

ScanLabeler::ScanLabeler()
{
    addTag(Tag::get("General"));
    addTag(Tag::get("Vision"));

}

ScanLabeler::~ScanLabeler()
{
}

void ScanLabeler::setupParameters()
{
    addParameter(param::ParameterFactory::declareTrigger("submit", param::ParameterDescription("Continue with the current labeling")),
                                                         boost::bind(&ScanLabeler::submit, this));

    addParameter(param::ParameterFactory::declareRange("label",
                                                       param::ParameterDescription("The label to be assigned to the selected points"),
                                                       0, 9, 0, 1));
}

void ScanLabeler::setup()
{    
    input_ = modifier_->addMultiInput<ScanMessage, LabeledScanMessage>("Scan");
    output_ = modifier_->addOutput<LabeledScanMessage>("Labeled Scan");
}

void ScanLabeler::submit()
{
    submit_request();
}

void ScanLabeler::process()
{
    InteractiveNode::process();

    result_.reset();

    if(input_->isMessage<LabeledScanMessage>()) {
        LabeledScanMessage::Ptr scan_msg = input_->getMessage<LabeledScanMessage>();
        display_request(&scan_msg->value);

    } else if(input_->isMessage<ScanMessage>()) {
        ScanMessage::Ptr scan_msg = input_->getMessage<ScanMessage>();
        display_request(&scan_msg->value);

    } else {
        throw std::runtime_error("invalid input type");
    }

    waitForView();

    output_->publish(result_);
}


void ScanLabeler::setResult(connection_types::LabeledScanMessage::Ptr result)
{
    result_ = result;
    done();
}
