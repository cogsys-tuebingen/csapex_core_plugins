/// HEADER
#include "render_labels.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/color.hpp>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;


CSAPEX_REGISTER_CLASS(vision_plugins::RenderLabels, csapex::Node)

RenderLabels::RenderLabels()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));
    addParameter(param::ParameterFactory::declareRange("color occupancy", 0.1, 1.0, 0.25, 0.05));
}

void RenderLabels::process()
{
#warning "FIX ENCODING"
    CvMatMessage::Ptr labels = labels_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr output(new CvMatMessage(enc::bgr));

    if(image_->isConnected()) {
        CvMatMessage::Ptr image = image_->getMessage<connection_types::CvMatMessage>();
        if(image->getEncoding() != enc::bgr)
            throw std::runtime_error("Image encoding must be 'bgr'!");
        output->value = image->value.clone();
    }

    std::map<unsigned short, cv::Vec3b> colors;
    cv::Mat label_colors(labels->value.rows, labels->value.cols, CV_8UC3, cv::Scalar(0));
    for(int y = 0 ; y < labels->value.rows ; ++y) {
        for(int x = 0 ; x < labels->value.cols ; ++x) {
            unsigned short label = labels->value.at<unsigned short>(y,x);
            if(label != 0) {
                if(colors.find(label) == colors.end()) {
                    double r,g,b;
                    color::fromCount(colors.size(), r,g,b);
                    colors.insert(std::make_pair(label, cv::Vec3b(b,g,r)));
                }
                label_colors.at<cv::Vec3b>(y,x) = colors.at(label);
            }
        }
    }

    if(output->value.empty()) {
        output->value = label_colors;
    } else {
        double occ = param<double>("color occupancy");
        cv::addWeighted(output->value, 1.0 - occ, label_colors, occ, 0.0, output->value);
    }
    output_->publish(output);
}

void RenderLabels::setup()
{
    labels_ = modifier_->addInput<CvMatMessage>("labels");
    image_  = modifier_->addInput<CvMatMessage>("image", true);
    output_ = modifier_->addOutput<CvMatMessage>("rendered");
}

