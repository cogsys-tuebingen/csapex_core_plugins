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

QIcon ScanLabeler::getIcon() const
{
    return QIcon(":/picture.png");
}

void ScanLabeler::setupParameters()
{
    addParameter(param::ParameterFactory::declareTrigger("submit"), boost::bind(&ScanLabeler::submit, this));
}

void ScanLabeler::setup()
{
    NodeModifier mod(this);
    input_ = mod.addMultiInput<ScanMessage, LabeledScanMessage>("Scan");
    output_ = mod.addOutput<LabeledScanMessage>("Labeled Scan");
}

void ScanLabeler::submit()
{
    std::cerr << "submit request" << std::endl;
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

    std::cerr << "wait" << std::endl;

    waitForView();

    std::cerr << "done" << std::endl;

    output_->publish(result_);
}


void ScanLabeler::setResult(connection_types::LabeledScanMessage::Ptr result)
{
    std::cerr << "set" << std::endl;
    result_ = result;
    done();
}
