/// HEADER
#include "matrix_to_heatmap.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <boost/assign/list_of.hpp>
#include <boost/function.hpp>

CSAPEX_REGISTER_CLASS(csapex::MatrixToHeatmap, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace {
const cv::Point3f red(0,0,255);                     /// P0
const cv::Point3f green(0,255,0);                   /// P1
const cv::Point3f blue(255,0,0);                    /// p3
const cv::Point3f fac1 = (blue - 2 * green + red);
const cv::Point3f fac2 = (-2*blue + 2 * green);

inline cv::Vec3f bezierColor(const float value)
{
    cv::Point3f  col = fac1 * value * value + fac2 * value + blue;
    return cv::Vec3f(std::floor(col.x + .5), std::floor(col.y + .5), std::floor(col.z + .5));
}

inline cv::Vec3f parabolaColor(const float value)
{
    return value * value * red + (value - 1) * (value - 1) * blue;
}

typedef boost::function<cv::Vec3f (float)> colorFunction;

inline void renderHeatmap(cv::Mat &src, cv::Mat &dst, colorFunction &color, const cv::Mat &mask)
{
    if(src.channels() > 1) {
        throw std::runtime_error("Single channel matrix required for rendering!");
    }

    if(src.type() != CV_32FC1)
        src.convertTo(src, CV_32FC1);

    double min;
    double max;
    cv::minMaxLoc(src, &min, &max, NULL, NULL, mask);
    src = src - (float) min;
    max -= min;
    src = src / (float) max;
    dst = cv::Mat(src.rows, src.cols, CV_32FC3, cv::Scalar::all(0));
    for(int i = 0 ; i < dst.rows ; ++i) {
        for(int j = 0 ; j < dst.cols ; ++j) {
            if(mask.at<uchar>(i,j) != 0)
                dst.at<cv::Vec3f>(i,j) = color(src.at<float>(i,j));
        }
    }
}
}

MatrixToHeatmap::MatrixToHeatmap() :
    color_type_(BEZIER)
{
    Tag::createIfNotExists("Visualization");
    addTag(Tag::get("Visualization"));
    addTag(Tag::get("Vision"));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("BEZIER", (int) BEZIER)
            ("PARABOLA", (int) PARABOLA);

    addParameter(param::ParameterFactory::declareParameterSet<int>("coloring", types),
                 boost::bind(&MatrixToHeatmap::update, this));

}

void MatrixToHeatmap::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));


    cv::Mat working = in->value.clone();
    cv::Mat heatmap (working.rows, working.cols, CV_32FC3, cv::Scalar::all(0));
    cv::Mat mean    (working.rows, working.cols, CV_32FC1, cv::Scalar::all(0));
    cv::Mat mask;
    if(mask_->isConnected()) {
        CvMatMessage::Ptr mask_msg = mask_->getMessage<connection_types::CvMatMessage>();
        mask = mask_msg->value;
    } else {
        mask = cv::Mat(working.rows, working.cols, CV_8UC1, 255);
    }

    std::vector<cv::Mat> channels;
    cv::split(working, channels);

    for(std::vector<cv::Mat>::iterator it = channels.begin() ; it != channels.end() ; ++it) {
        it->convertTo(*it, CV_32FC1);
        mean += *it;
    }

    float divider = 1 / (float) channels.size();
    mean = mean * divider;

    colorFunction fc;
    switch(color_type_) {
    case BEZIER:
        fc = &bezierColor;
        break;
    case PARABOLA:
        fc = &parabolaColor;
        break;
    default:
        throw std::runtime_error("Unknown color function type!");
    }

    renderHeatmap(mean, heatmap, fc, mask);

    heatmap.convertTo(heatmap, CV_8UC3);
    out->value = heatmap;
    output_->publish(out);
}

void MatrixToHeatmap::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Matrix");
    output_ = addOutput<CvMatMessage>("Extrema");
    mask_   = addInput<CvMatMessage>("Mask",true);

    update();
}

void MatrixToHeatmap::update()
{
    color_type_ = (ColorType) param<int>("coloring");
}
