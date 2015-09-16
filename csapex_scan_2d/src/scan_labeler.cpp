/// HEADER
#include "scan_labeler.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

CSAPEX_REGISTER_CLASS(csapex::ScanLabeler, csapex::Node)


using namespace csapex;
using namespace connection_types;
using namespace lib_laser_processing;

ScanLabeler::ScanLabeler()
{
}

ScanLabeler::~ScanLabeler()
{
}

void ScanLabeler::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("submit", csapex::param::ParameterDescription("Continue with the current labeling")),
                                                         std::bind(&ScanLabeler::submit, this));

    addParameter(csapex::param::ParameterFactory::declareBool("automatic", csapex::param::ParameterDescription("Automatically continue without user interaction"), false));
    addParameter(csapex::param::ParameterFactory::declareRange("label",
                                                       csapex::param::ParameterDescription("The label to be assigned to the selected points"),
                                                       0, 9, 0, 1));
}

void ScanLabeler::setup(NodeModifier& node_modifier)
{    
    input_ = node_modifier.addMultiInput<ScanMessage, LabeledScanMessage>("Scan");
    output_ = node_modifier.addOutput<LabeledScanMessage>("Labeled Scan");
}

void ScanLabeler::submit()
{
    submit_request();
}

void ScanLabeler::beginProcess()
{
    result_.reset();

    if(msg::isMessage<LabeledScanMessage>(input_)) {
        LabeledScanMessage::ConstPtr scan_msg = msg::getMessage<LabeledScanMessage>(input_);
        display_request(&scan_msg->value);

    } else if(msg::isMessage<ScanMessage>(input_)) {
        ScanMessage::ConstPtr scan_msg = msg::getMessage<ScanMessage>(input_);
        display_request(&scan_msg->value);

    } else {
        throw std::runtime_error("invalid input type");
    }
}

void ScanLabeler::finishProcess()
{
    msg::publish(output_, result_);
}

void ScanLabeler::setResult(connection_types::LabeledScanMessage::Ptr result)
{
    result_ = result;
    done();
}
