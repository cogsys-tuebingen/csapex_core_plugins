/// HEADER
#include "image_padding.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ImagePadding, csapex::Node)


using namespace csapex;
using namespace connection_types;

ImagePadding::ImagePadding()
{
}

void ImagePadding::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("border", 0, 1000, 0, 1));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("mask offset", 0, 100, 0, 1));
    parameters.addParameter(csapex::param::ParameterFactory::declareColorParameter("color", 0x00, 0x00, 0x00));
}

void ImagePadding::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("Image");

    output_ = node_modifier.addOutput<CvMatMessage>("Expanded Image");
    output_mask_ = node_modifier.addOutput<CvMatMessage>("Expanded Mask");
}

void ImagePadding::process()
{
    if(!msg::isConnected(output_) && !msg::isConnected(output_mask_)) {
        return;
    }

    CvMatMessage::ConstPtr img_msg = msg::getMessage<CvMatMessage>(input_);

    int rows = img_msg->value.rows;
    int cols = img_msg->value.cols;

    int border = readParameter<int>("border");

    if(msg::isConnected(output_)) {
        CvMatMessage::Ptr result(new CvMatMessage(img_msg->getEncoding(), img_msg->stamp_micro_seconds));
        const std::vector<int>& c = readParameter<std::vector<int> >("color");
        cv::Scalar color(c[2], c[1], c[0]);
        result->value = cv::Mat(rows + 2 * border, cols + 2 * border, img_msg->value.type(), color);
        cv::Mat roi(result->value, cv::Rect(border, border, cols, rows));

        img_msg->value.copyTo(roi);

        msg::publish(output_, result);
    }

    if(msg::isConnected(output_mask_)) {
        CvMatMessage::Ptr result(new CvMatMessage(enc::mono, img_msg->stamp_micro_seconds));

        result->value = cv::Mat(rows + 2 * border, cols + 2 * border, CV_8UC1, cv::Scalar::all(0));

        int mask_offset = readParameter<int>("mask offset");
        cv::Rect roi_rect(border+mask_offset, border+mask_offset, cols-2*mask_offset, rows-2*mask_offset);
        cv::rectangle(result->value, roi_rect, cv::Scalar::all(255), CV_FILLED);

        msg::publish(output_mask_, result);
    }

}
