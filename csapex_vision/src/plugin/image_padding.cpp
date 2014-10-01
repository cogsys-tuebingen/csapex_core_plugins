/// HEADER
#include "image_padding.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ImagePadding, csapex::Node)


using namespace csapex;
using namespace connection_types;

ImagePadding::ImagePadding()
{
}

void ImagePadding::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("border", 0, 1000, 0, 1));
    addParameter(param::ParameterFactory::declareRange("mask offset", 0, 100, 0, 1));
    addParameter(param::ParameterFactory::declareColorParameter("color", 0x00, 0x00, 0x00));
}

void ImagePadding::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("Image");

    output_ = modifier_->addOutput<CvMatMessage>("Expanded Image");
    output_mask_ = modifier_->addOutput<CvMatMessage>("Expanded Mask");
}

void ImagePadding::process()
{
    if(!output_->isConnected() && !output_mask_->isConnected()) {
        return;
    }

    CvMatMessage::Ptr img_msg = input_->getMessage<CvMatMessage>();

    int rows = img_msg->value.rows;
    int cols = img_msg->value.cols;

    int border = readParameter<int>("border");

    if(output_->isConnected()) {
        CvMatMessage::Ptr result(new CvMatMessage(img_msg->getEncoding(), img_msg->stamp));
        const std::vector<int>& c = readParameter<std::vector<int> >("color");
        cv::Scalar color(c[2], c[1], c[0]);
        result->value = cv::Mat(rows + 2 * border, cols + 2 * border, img_msg->value.type(), color);
        cv::Mat roi(result->value, cv::Rect(border, border, cols, rows));

        img_msg->value.copyTo(roi);

        output_->publish(result);
    }

    if(output_mask_->isConnected()) {
        CvMatMessage::Ptr result(new CvMatMessage(enc::mono, img_msg->stamp));

        result->value = cv::Mat(rows + 2 * border, cols + 2 * border, CV_8UC1, cv::Scalar::all(0));

        int mask_offset = readParameter<int>("mask offset");
        cv::Rect roi_rect(border+mask_offset, border+mask_offset, cols-2*mask_offset, rows-2*mask_offset);
        cv::rectangle(result->value, roi_rect, cv::Scalar::all(255), CV_FILLED);

        output_mask_->publish(result);
    }

}
