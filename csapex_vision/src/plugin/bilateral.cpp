/// HEADER
#include "bilateral.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::BilateralFilter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace {
void bilateralFilter(const cv::Mat &src, cv::Mat &dst,
                     const int d, const double sigmaColor,
                     const double sigmaSpace, int borderType = cv::BORDER_DEFAULT)
{
    cv::Mat tmp = src.clone();
    int type  = src.type();
    int ctype = type & 7;
    if(type == CV_8UC1  || type == CV_8UC3 ||
       type == CV_32FC1 || type == CV_32FC3) {
       cv::bilateralFilter(tmp, dst, d, sigmaColor, sigmaSpace, borderType);
    } else if(src.channels() == 1) {
        tmp.convertTo(tmp, CV_32FC1);
        cv::bilateralFilter(tmp, dst, d, sigmaColor, sigmaSpace, borderType);
        dst.convertTo(dst, type);
    } else if(ctype == CV_8U || ctype == CV_32F) {
        std::vector<cv::Mat> channels;
        cv::split(tmp, channels);
        for(std::vector<cv::Mat>::iterator it = channels.begin() ; it != channels.end() ; ++it) {
            cv::Mat buff;
            cv::bilateralFilter(*it, buff, d, sigmaColor, sigmaSpace, borderType);
            buff.copyTo(*it);
        }
        cv::merge(channels,dst);
    } else {
        std::vector<cv::Mat> channels;
        cv::split(tmp, channels);
        for(std::vector<cv::Mat>::iterator it = channels.begin() ; it != channels.end() ; ++it) {
            it->convertTo(*it, CV_32FC1);
            cv::Mat buff;
            cv::bilateralFilter(*it, buff, d, sigmaColor, sigmaSpace, borderType);
            buff.convertTo(*it, ctype);
        }
        cv::merge(channels,dst);
    }

}
}

BilateralFilter::BilateralFilter() :
    d_(1),
    sigma_color_(0.0),
    sigma_space_(0.0)
{
}

void BilateralFilter::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("d", 1, 255, d_, 1));
    parameters.addParameter(param::ParameterFactory::declareRange("sigma color", -255.0, 255.0, sigma_color_, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("sigma space", -255.0, 255.0, sigma_space_, 0.1));
}

void BilateralFilter::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));

    d_           = readParameter<int>("d");
    sigma_color_ = readParameter<double>("sigma color");
    sigma_space_ = readParameter<double>("sigma space");

    /// WORKAROUND
    bilateralFilter(in->value, out->value, d_, sigma_color_, sigma_space_);
    msg::publish(output_, out);
}

void BilateralFilter::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("Unblurred");
    output_ = node_modifier.addOutput<CvMatMessage>("Blurred");
}
