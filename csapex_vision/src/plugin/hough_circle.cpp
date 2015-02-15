/// HEADER
#include "hough_circle.h"

/// COMPONENT
#include <csapex_vision/cv_mat_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::HoughCircle, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

HoughCircle::HoughCircle()
{
    std::map<std::string, int> methods;
    methods["CV_HOUGH_GRADIENT"] = (int) CV_HOUGH_GRADIENT;
    methods["CV_HOUGH_STANDARD"] = (int) CV_HOUGH_STANDARD;
    methods["CV_HOUGH_PROBABILISTIC"] = (int) CV_HOUGH_PROBABILISTIC;
    methods["CV_HOUGH_MULTI_SCALE"] = (int) CV_HOUGH_MULTI_SCALE;
    addParameter(param::ParameterFactory::declareParameterSet("method", methods, (int) CV_HOUGH_STANDARD));

    addParameter(param::ParameterFactory::declareRange<double>("dp", 0.01, 10.00, 1.0, 0.01));
    addParameter(param::ParameterFactory::declareRange<double>("minDist", 0.0, 800.0, 100.0, 0.1));
    addParameter(param::ParameterFactory::declareRange<double>("param1", 0.00, 500.0, 200.0, 0.1));
    addParameter(param::ParameterFactory::declareRange<double>("param2", 0.00, 500.0, 100.0, 0.1));
    addParameter(param::ParameterFactory::declareRange<int>("minRadius", 0, 100, 0, 1));
    addParameter(param::ParameterFactory::declareRange<int>("maxRadius", 0, 100, 0, 1));
}

void HoughCircle::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("Image");
    output_ = modifier_->addOutput<CvMatMessage>("Debug Image");
}

void HoughCircle::process()
{
    CvMatMessage::ConstPtr msg = msg::getMessage<CvMatMessage>(input_);

    int method = readParameter<int>("method");
    double dp = readParameter<double>("dp");
    double minDist = readParameter<double>("minDist");
    double param1 = readParameter<double>("param1");
    double param2 = readParameter<double>("param2");
    int minRadius = readParameter<int>("minRadius");
    int maxRadius = readParameter<int>("maxRadius");

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(msg->value, circles, method, dp, minDist, param1, param2, minRadius, maxRadius);

    CvMatMessage::Ptr out(new CvMatMessage(msg->getEncoding(), msg->stamp_micro_seconds));
    if(!msg->hasChannels(1, CV_8U)) {
        cv::cvtColor(msg->value, out->value, CV_GRAY2BGR);
    } else {
        msg->value.copyTo(out->value);
    }

    for(unsigned i = 0; i < circles.size(); ++i) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(out->value, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        cv::circle(out->value, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

    msg::publish(output_, out);
}

