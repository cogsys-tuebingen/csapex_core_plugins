/// HEADER
#include "scan_renderer.h"

/// PROJECT
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/profiling/timer.h>
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

void ScanRenderer::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addMultiInput<ScanMessage, LabeledScanMessage>("Scan");
    output_ = node_modifier.addOutput<CvMatMessage>("Render");
}

void ScanRenderer::process()
{
    if(msg::isMessage<LabeledScanMessage>(input_)) {
        LabeledScanMessage::ConstPtr scan_msg = msg::getMessage<LabeledScanMessage>(input_);
        doProcess<LabeledScan>(scan_msg->value);

    } else if(msg::isMessage<ScanMessage>(input_)) {
        ScanMessage::ConstPtr scan_msg = msg::getMessage<ScanMessage>(input_);
        doProcess<Scan>(scan_msg->value);

    } else {
        throw std::runtime_error("invalid input type");
    }
}

template <typename ScanType>
void ScanRenderer::doProcess(const ScanType& scan)
{
    CvMatMessage::Ptr output(new CvMatMessage(enc::bgr, 0));

    renderer.render(scan, output->value);

    msg::publish(output_, output);
}
