/// HEADER
#include "gaussian_blur.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>


CSAPEX_REGISTER_CLASS(csapex::GaussianBlur, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

GaussianBlur::GaussianBlur() :
    kernel_(1),
    sigma_x_(0.1),
    sigma_y_(0.0)
{
}

void GaussianBlur::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("kernel", 1, 255, kernel_, 2),
                 std::bind(&GaussianBlur::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("sigma x", 0.1, 128.0, sigma_x_, 0.1),
                 std::bind(&GaussianBlur::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("sigma y", 0.0, 128.0, sigma_y_, 0.1),
                 std::bind(&GaussianBlur::update, this));
}

void GaussianBlur::process()
{
    CvMatMessage::ConstPtr in_msg = msg::getMessage<connection_types::CvMatMessage>(input_mat_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in_msg->getEncoding(), in_msg->frame_id, in_msg->stamp_micro_seconds));

    CvMatMessage::ConstPtr mask_msg;
    if (msg::hasMessage(input_mask_))
        mask_msg = msg::getMessage<CvMatMessage>(input_mask_);

    if (!mask_msg)
        cv::GaussianBlur(in_msg->value, out->value, cv::Size(kernel_, kernel_), sigma_x_, sigma_y_);
    else
    {
        cv::Mat im;
        cv::Mat mask;
        in_msg->value.convertTo(im, CV_32FC(in_msg->value.channels()), 1.0 / 255.0);
        if (in_msg->value.channels() == 3)
        {
            cv::cvtColor(mask_msg->value, mask, cv::COLOR_GRAY2BGR);
            mask.convertTo(mask, CV_32FC3, 1.0 / 255.0);
        }
        else
            mask_msg->value.convertTo(mask, CV_32FC1, 1.0 / 255.0);

        cv::Mat blurred;
        cv::GaussianBlur(im, blurred, cv::Size(kernel_, kernel_), sigma_x_, sigma_y_);
        cv::GaussianBlur(mask, mask, cv::Size(kernel_, kernel_), sigma_x_, sigma_y_);

        cv::Mat tmp, tmp2;
        cv::subtract(cv::Scalar::all(1.0), mask, tmp);
        cv::multiply(tmp, im, tmp);
        cv::multiply(mask, blurred, tmp2);
        cv::add(tmp, tmp2, out->value);

        out->value.convertTo(out->value, CV_8UC1, 255.0);
    }
    msg::publish(output_, out);
}

void GaussianBlur::setup(NodeModifier& node_modifier)
{
    input_mat_ = node_modifier.addInput<CvMatMessage>("Unblurred");
    input_mask_ = node_modifier.addOptionalInput<CvMatMessage>("Mask");
    output_ = node_modifier.addOutput<CvMatMessage>("Blurred");

    update();
}

void GaussianBlur::update()
{
    kernel_  = readParameter<int>("kernel");
    sigma_x_ = readParameter<double>("sigma x");
    sigma_y_ = readParameter<double>("sigma y");
}
