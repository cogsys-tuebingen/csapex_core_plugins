/// HEADER
#include "scan_renderer.h"

/// PROJECT
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/timer.h>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::ScanRenderer, csapex::Node)


ScanRenderer::ScanRenderer()
{
    setTemporaryParameters(renderer.getParameters());

    renderer.useTimer(profiling_timer_);
}

void ScanRenderer::setup()
{
    input_ = modifier_->addMultiInput<ScanMessage, LabeledScanMessage>("Scan");
    output_ = modifier_->addOutput<CvMatMessage>("Render");
}

void ScanRenderer::process()
{
    if(input_->isMessage<LabeledScanMessage>()) {
        LabeledScanMessage::Ptr scan_msg = input_->getMessage<LabeledScanMessage>();
        doProcess<LabeledScan>(scan_msg->value);

    } else if(input_->isMessage<ScanMessage>()) {
        ScanMessage::Ptr scan_msg = input_->getMessage<ScanMessage>();
        doProcess<Scan>(scan_msg->value);

    } else {
        throw std::runtime_error("invalid input type");
    }
}

template <typename ScanType>
void ScanRenderer::doProcess(ScanType& scan)
{
    CvMatMessage::Ptr output(new CvMatMessage(enc::bgr));

    renderer.render(scan, output->value);

    output_->publish(output);
}
