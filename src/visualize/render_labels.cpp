/// HEADER
#include "render_labels.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;


CSAPEX_REGISTER_CLASS(vision_plugins::RenderLabels, csapex::Node)

namespace {
#define _HSV2RGB_(H, S, V, R, G, B) \
    { \
    double _h = H/60.; \
    int _hf = (int)floor(_h); \
    int _hi = ((int)_h)%6; \
    double _f = _h - _hf; \
    \
    double _p = V * (1. - S); \
    double _q = V * (1. - _f * S); \
    double _t = V * (1. - (1. - _f) * S); \
    \
    switch (_hi) \
    { \
    case 0: \
    R = 255.*V; G = 255.*_t; B = 255.*_p; \
    break; \
    case 1: \
    R = 255.*_q; G = 255.*V; B = 255.*_p; \
    break; \
    case 2: \
    R = 255.*_p; G = 255.*V; B = 255.*_t; \
    break; \
    case 3: \
    R = 255.*_p; G = 255.*_q; B = 255.*V; \
    break; \
    case 4: \
    R = 255.*_t; G = 255.*_p; B = 255.*V; \
    break; \
    case 5: \
    R = 255.*V; G = 255.*_p; B = 255.*_q; \
    break; \
} \
}
}


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
                    _HSV2RGB_((double) ((colors.size() * 77) % 360), 1.0, 1.0, r, g, b);
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
    setSynchronizedInputs(true);
    labels_ = addInput<CvMatMessage>("Labels");
    image_  = addInput<CvMatMessage>("Image", true);
    output_ = addOutput<CvMatMessage>("Rendered");
}

