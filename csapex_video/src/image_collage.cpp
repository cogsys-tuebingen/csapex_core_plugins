/// HEADER
#include "image_collage.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::ImageCollage, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


ImageCollage::ImageCollage()
{
}

void ImageCollage::setupParameters()
{
    p_x_ = boost::dynamic_pointer_cast<param::RangeParameter>
            (param::ParameterFactory::declareRange("x",
                                                 param::ParameterDescription("x position of the overlay"),
                                                 0, 800, 0, 1));
    p_y_ = boost::dynamic_pointer_cast<param::RangeParameter>
            (param::ParameterFactory::declareRange("y",
                                                   param::ParameterDescription("y position of the overlay"),
                                                   0, 600, 0, 1));

    addParameter(p_x_);
    addParameter(p_y_);
}

void ImageCollage::setup()
{
    in_main_  = modifier_->addInput<CvMatMessage>("Main image");
    in_secondary_  = modifier_->addInput<CvMatMessage>("Secondary image");

    out_ = modifier_->addOutput<CvMatMessage>("Collage");
}

void ImageCollage::process()
{
    CvMatMessage::ConstPtr primary = in_main_->getMessage<CvMatMessage>();
    CvMatMessage::ConstPtr secondary = in_secondary_->getMessage<CvMatMessage>();

    CvMatMessage::Ptr output(new CvMatMessage(primary->getEncoding(), primary->stamp));

    primary->value.copyTo(output->value);


    int w_1 = primary->value.cols;
    int h_1 = primary->value.rows;

    int w_2 = secondary->value.cols;
    int h_2 = secondary->value.rows;

    int max_x = std::max(0, w_1 - w_2);
    int max_y = std::max(0, h_1 - h_2);

    if(p_x_->max<int>() != max_x) {
        p_x_->setMax(max_x);
    }
    if(p_y_->max<int>() != max_y) {
        p_y_->setMax(max_y);
    }

    int x = readParameter<int>("x");
    int y = readParameter<int>("y");

    int w = std::min(w_2, w_1 - x - 1);
    int h = std::min(h_2, h_1 - y - 1);

    cv::Mat source(secondary->value, cv::Rect(0, 0, w, h));
    cv::Mat target(output->value, cv::Rect(x, y, w, h));

    source.copyTo(target);

    out_->publish(output);
}

