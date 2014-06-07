/// HEADER
#include "scan_renderer.h"

/// PROJECT
#include <csapex_core_plugins/ros_message_conversion.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/timer.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::ScanRenderer, csapex::Node)


ScanRenderer::ScanRenderer()
{
    addTag(Tag::get("Laser"));
    addTag(Tag::get("Scan"));

    setTemporaryParameters(renderer.getParameters());
}

void ScanRenderer::setup()
{
    input_ = modifier_->addMultiInput<ScanMessage, LabeledScanMessage>("Scan");
    output_ = modifier_->addOutput<CvMatMessage>("Render");
}

namespace {
void drawRays(const Scan& scan, cv::Mat& img, const cv::Point2f& origin, cv::Scalar color, double angle_offset, double scale)
{
    float angle = scan.angle_min + angle_offset;
    float angle_step = scan.angle_increment;
    for(std::vector<LaserBeam>::const_iterator it = scan.rays.begin(); it != scan.rays.end(); ++it) {
        const LaserBeam& range = *it;

        cv::Point2f pt(range.pos(0), range.pos(1));

        cv::line(img, origin, origin + pt * scale, color, std::ceil(scale / 5.0), CV_AA);

        angle += angle_step;
    }
}
void drawHits(const Scan& scan, cv::Mat& img, const cv::Point2f& origin, cv::Scalar color, cv::Scalar, double angle_offset, double scale)
{
    float angle = scan.angle_min + angle_offset;
    float angle_step = scan.angle_increment;
    for(std::vector<LaserBeam>::const_iterator it = scan.rays.begin(); it != scan.rays.end(); ++it) {
        const LaserBeam& range = *it;

        cv::Point2f pt(range.pos(0), range.pos(1));

        cv::circle(img, origin + pt * scale, std::ceil(scale / 10.0), color, CV_FILLED, CV_AA);

        angle += angle_step;
    }
}
void drawHits(const LabeledScan& scan, cv::Mat& img, const cv::Point2f& origin, cv::Scalar color, cv::Scalar marked, double angle_offset, double scale)
{
    float angle = scan.angle_min + angle_offset;
    float angle_step = scan.angle_increment;

    std::vector<LaserBeam>::const_iterator range_it = scan.rays.begin();
    std::vector<int>::const_iterator label_it = scan.labels.begin();

    for(; range_it != scan.rays.end(); ++range_it, ++label_it) {
        const LaserBeam& range = *range_it;

        int label = *label_it;

        cv::Point2f pt(range.pos(0), range.pos(1));

        cv::circle(img, origin + pt * scale, std::ceil(scale / 10.0), label != 0 ? marked : color, CV_FILLED, CV_AA);

        angle += angle_step;
    }
}
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

    int w = param<int>("width");
    int h = param<int>("height");

    const std::vector<int>& color = param<std::vector<int> >("color/bg");
    cv::Scalar bgColor(color[2], color[1], color[0]);

    output->value = cv::Mat(h, w, CV_8UC3, bgColor);

    cv::Point2f origin(w/2, h/2);
    double scale = param<double>("scale") * 10.0;
    double angle = param<double>("rotation");

    if(param<bool>("drawRays")) {
        INTERLUDE("draw rays");

        const std::vector<int>& color = param<std::vector<int> >("color/ray");
        cv::Scalar rayColor(color[2], color[1], color[0]);
        drawRays(scan, output->value, origin, rayColor, angle, scale);
    }
    if(param<bool>("drawHits")) {
        INTERLUDE("draw hits");

        const std::vector<int>& color = param<std::vector<int> >("color/hit");
        const std::vector<int>& marked = param<std::vector<int> >("color/marked");
        cv::Scalar hitColor(color[2], color[1], color[0]);
        cv::Scalar markedColor(marked[2], marked[1], marked[0]);
        drawHits(scan, output->value, origin, hitColor, markedColor, angle, scale);
    }

    output_->publish(output);
}
