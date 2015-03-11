/// HEADER
#include "matrix_to_heatmap.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <utils_cv/heatmap.hpp>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <utils_cv/color_functions.hpp>


/// SYSTEM
#include <boost/assign/list_of.hpp>
#include <functional>

CSAPEX_REGISTER_CLASS(vision_plugins::MatrixToHeatmap, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

MatrixToHeatmap::MatrixToHeatmap() :
    color_type_(BEZIER)
{
}

void MatrixToHeatmap::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));


    cv::Mat working = in->value.clone();
    cv::Mat heatmap (working.rows, working.cols, CV_32FC3, cv::Scalar::all(0));
    cv::Mat mean    (working.rows, working.cols, CV_32FC1, cv::Scalar::all(0));
    cv::Mat mask;
    if(msg::hasMessage(mask_)) {
        CvMatMessage::ConstPtr mask_msg = msg::getMessage<connection_types::CvMatMessage>(mask_);
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

    utils_cv::Heatmap::colorFunction fc;
    switch(color_type_) {
    case BEZIER:
        fc = &utils_cv::color::bezierColor<cv::Vec3f>;
        break;
    case PARABOLA:
        fc = &utils_cv::color::parabolaColor<cv::Vec3f>;
        break;
    default:
        throw std::runtime_error("Unknown color function type!");
    }

    utils_cv::Heatmap::renderHeatmap(mean, heatmap, fc, mask);

    out->value = heatmap;
    msg::publish(output_, out);
}

void MatrixToHeatmap::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("matrix");
    output_ = node_modifier.addOutput<CvMatMessage>("heatmap");
    mask_   = node_modifier.addOptionalInput<CvMatMessage>("mask");

    update();
}

void MatrixToHeatmap::setupParameters(Parameterizable& parameters)
{
    std::map<std::string, int> types = boost::assign::map_list_of
            ("BEZIER", (int) BEZIER)
            ("PARABOLA", (int) PARABOLA);

    parameters.addParameter(param::ParameterFactory::declareParameterSet<int>("coloring", types, (int) BEZIER),
                 std::bind(&MatrixToHeatmap::update, this));
}

void MatrixToHeatmap::update()
{
    color_type_ = (ColorType) readParameter<int>("coloring");
}
