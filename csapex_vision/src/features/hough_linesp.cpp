/// HEADER
#include "hough_linesp.h"

/// PROJECT
#include <QColor>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/utility/color.hpp>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/yaml_io.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::HoughLinesP, csapex::Node)

// double rho, double theta, int threshold, double minLineLength=0, double
// maxLineGap=0

HoughLinesP::HoughLinesP() : rho_(1.), theta_(CV_PI), threshold_(80), min_line_length_(30), max_line_gap_(10)
{
}

void HoughLinesP::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);

    if (!in->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::bgr, in->frame_id, in->stamp_micro_seconds));
    cv::cvtColor(in->value, out->value, cv::COLOR_GRAY2BGR);

    std::shared_ptr<std::vector<cv::Vec4i>> lines_ptr(new std::vector<cv::Vec4i>);
    std::vector<cv::Vec4i>& lines = *lines_ptr;
    cv::HoughLinesP(in->value, lines, rho_, theta_ / 180, threshold_, min_line_length_, max_line_gap_);

    for (size_t i = 0; i < lines.size(); i++) {
        double b, g, r;
        color::fromCount(i, r, g, b);
        cv::line(out->value, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(b, g, r), 1, cv::LINE_AA);
    }

    msg::publish<GenericVectorMessage, cv::Vec4i>(output_vector_, lines_ptr);
    msg::publish(output_, out);
}

void HoughLinesP::setup(NodeModifier& node_modifier)
{
    CornerLineDetection::setup(node_modifier);
    output_vector_ = node_modifier.addOutput<GenericVectorMessage, cv::Vec4i>("lines saved in vector");
    update();
}

void HoughLinesP::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("rho", 0.1, 100.0, 1.0, 0.1), std::bind(&HoughLinesP::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("theta", 0.1, 2 * CV_PI, CV_PI, 0.01), std::bind(&HoughLinesP::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("threshold", 1, 500, 80, 1), std::bind(&HoughLinesP::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("min length", 0.0, 1000.0, 30.0, 1.0), std::bind(&HoughLinesP::update, this));
    parameters.addParameter(csapex::param::factory::declareRange("max gap", 0.0, 400.0, 10.0, 1.0), std::bind(&HoughLinesP::update, this));
}

void HoughLinesP::update()
{
    rho_ = readParameter<double>("rho");
    theta_ = readParameter<double>("theta");
    threshold_ = readParameter<int>("threshold");
    min_line_length_ = readParameter<double>("min length");
    max_line_gap_ = readParameter<double>("max gap");
}
