/// HEADER
#include "image_collage.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::ImageCollage, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


ImageCollage::ImageCollage()
{
}

void ImageCollage::setupParameters(Parameterizable& parameters)
{
    csapex::param::Parameter::Ptr x = csapex::param::factory::declareRange("x",
                                                   csapex::param::ParameterDescription("x position of the overlay"),
                                                   0, 800, 0, 1);
    csapex::param::Parameter::Ptr y = csapex::param::factory::declareRange("y",
                                                   csapex::param::ParameterDescription("y position of the overlay"),
                                                   0, 600, 0, 1);

    p_x_ = std::dynamic_pointer_cast<param::RangeParameter>(x);
    p_y_ = std::dynamic_pointer_cast<param::RangeParameter>(y);

    parameters.addParameter(p_x_);
    parameters.addParameter(p_y_);
}

void ImageCollage::setup(NodeModifier& node_modifier)
{
    in_main_  = node_modifier.addInput<CvMatMessage>("Main image");
    in_secondary_  = node_modifier.addInput<CvMatMessage>("Secondary image");

    out_ = node_modifier.addOutput<CvMatMessage>("Collage");
}

void ImageCollage::process()
{
    CvMatMessage::ConstPtr primary = msg::getMessage<CvMatMessage>(in_main_);
    CvMatMessage::ConstPtr secondary = msg::getMessage<CvMatMessage>(in_secondary_);

    CvMatMessage::Ptr output(new CvMatMessage(primary->getEncoding(), primary->frame_id, primary->stamp_micro_seconds));

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

    msg::publish(out_, output);
}

