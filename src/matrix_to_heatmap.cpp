/// HEADER
#include "matrix_to_heatmap.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::MatrixToHeatMap, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace {
const cv::Point3f red(0,0,255);                     /// P0
const cv::Point3f green(0,255,255);                 /// P1
const cv::Point3f blue(255,0,0);                    /// p3
const cv::Point3f fac1 = (blue - 2 * green + red);
const cv::Point3f fac2 = (-2*blue + 2 * green);

inline cv::Vec3f maximumColor(const float value)
{
    cv::Point3f  col = fac1 * value * value + fac2 * value + blue;
    return cv::Vec3f(std::floor(col.x + .5), std::floor(col.y + .5), std::floor(col.z + .5));
}

inline void renderHeatmap(cv::Mat &src, cv::Mat &dst)
{
    if(src.channels() > 1) {
        throw std::runtime_error("Single channel matrix required for rendering!");
    }

    double min;
    double max;
    cv::minMaxLoc(src, &min, &max);
    cv::normalize(src, src, max);
    src.convertTo(src, CV_32FC1);

    dst = cv::Mat(src.rows, src.cols, CV_32FC3, cv::Scalar::all(0));
    for(int i = 0 ; i < dst.rows ; ++i) {
        for(int j = 0 ; j < dst.cols ; ++j) {
            dst.at<cv::Vec3f>(i,j) = maximumColor(src.at<float>(i,j));
        }
    }
}
}

MatrixToHeatMap::MatrixToHeatMap()
{
    Tag::createIfNotExists("Visualization");
    addTag(Tag::get("Visualization"));
    addTag(Tag::get("Vision"));
}

void MatrixToHeatMap::allConnectorsArrived()
{
    throw std::runtime_error("Not tested yet!");

    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    cv::Mat working = in->value.clone();
    cv::Mat heatmap = cv::Mat(working.rows, working.cols, CV_32FC3, cv::Scalar::all(0));
    std::vector<cv::Mat> channels;
    cv::split(working, channels);

    for(std::vector<cv::Mat>::iterator it = channels.begin() ; it != channels.end() ; ++it) {
        cv::Mat channel_heatmap;
        renderHeatmap(*it, channel_heatmap);
        heatmap += channel_heatmap;
    }

///    cv::Mat scale = cv::Mat(heatmap.rows, heatmap.cols, CV_32FC3, cv::Scalar::all(channels.size()));
///    cv::divide(heatmap, scale, heatmap);
    cv::divide(channels.size(), heatmap, heatmap);
    out->value = heatmap;
    output_->publish(out);
}

void MatrixToHeatMap::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Matrix");
    output_ = addOutput<CvMatMessage>("Extrema");

    update();
}

void MatrixToHeatMap::update()
{

}
