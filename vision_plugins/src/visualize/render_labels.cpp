/// HEADER
#include "render_labels.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;


CSAPEX_REGISTER_CLASS(vision_plugins::RenderLabels, csapex::Node)

RenderLabels::RenderLabels()
{
}

void RenderLabels::process()
{
    CvMatMessage::ConstPtr labels = msg::getMessage<connection_types::CvMatMessage>(labels_);
    CvMatMessage::Ptr output(new CvMatMessage(enc::bgr, labels->stamp_micro_seconds));

    if(msg::hasMessage(image_)) {
        CvMatMessage::ConstPtr image = msg::getMessage<connection_types::CvMatMessage>(image_);
        if(!image->hasChannels(3, CV_8U))
            throw std::runtime_error("Image encoding must be 8UC3!");
        output->value = image->value.clone();
    }

    std::map<unsigned short, cv::Vec3b> colors;
    cv::Mat label_colors(labels->value.rows, labels->value.cols, CV_8UC3, cv::Scalar(0));
    for(int y = 0 ; y < labels->value.rows ; ++y) {
        for(int x = 0 ; x < labels->value.cols ; ++x) {
            unsigned short label = labels->value.at<unsigned short>(y,x);
            if(label != 0) {
                if(colors.find(label) == colors.end()) {
                    double r = 0,g = 0,b = 0;
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
        double occ = readParameter<double>("color occupancy");
        cv::addWeighted(output->value, 1.0 - occ, label_colors, occ, 0.0, output->value);
    }
    msg::publish(output_, output);
}

void RenderLabels::setup(NodeModifier& node_modifier)
{
    labels_ = node_modifier.addInput<CvMatMessage>("labels");
    image_  = node_modifier.addOptionalInput<CvMatMessage>("image");
    output_ = node_modifier.addOutput<CvMatMessage>("rendered");
}

void RenderLabels::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("color occupancy", 0.1, 1.0, 0.25, 0.05));
}

