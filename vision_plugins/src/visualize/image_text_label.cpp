/// HEADER
#include "image_text_label.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <boost/assign.hpp>

using namespace csapex;
using namespace connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::ImageTextLabel, csapex::Node)


ImageTextLabel::ImageTextLabel()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));
}

void ImageTextLabel::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("Image");
    output_ = modifier_->addOutput<CvMatMessage>("Labeled");
}

void ImageTextLabel::setupParameters()
{

    std::map<std::string, int> positions = boost::assign::map_list_of
            ("BOTTOM_LEFT", BOTTOM_LEFT)
            ("BOTTOM_RIGHT", BOTTOM_RIGHT)
            ("TOP_LEFT", TOP_LEFT)
            ("TOP_RIGHT", TOP_RIGHT);
    addParameter(param::ParameterFactory::declareParameterSet("position", positions));
    addParameter(param::ParameterFactory::declareColorParameter("color/label", 255, 255, 255));
    addParameter(param::ParameterFactory::declareColorParameter("color/box", 0, 0, 0));
    addParameter(param::ParameterFactory::declareBool("boxed", false));
    addParameter(param::ParameterFactory::declareRange("thickness", 1, 10, 1, 1));
    addParameter(param::ParameterFactory::declareRange("scale", 1.0, 10.0, 1.0, 0.1));
    addParameter(param::ParameterFactory::declareText("label", "label"));

}

void ImageTextLabel::process()
{
    CvMatMessage::Ptr input  = input_->getMessage<CvMatMessage>();
    if(input->value.type() != CV_8UC3 && input->value.type() != CV_8UC1)
        throw std::runtime_error("Must be mono or color image!");

    CvMatMessage::Ptr output(new CvMatMessage(input->getEncoding()));
    output->value = input->value.clone();

    std::string label       = param<std::string>("label");
    double      font_scale  = param<double>("scale");
    int         thickness   = param<int>("thickness");
    Position    pos         = (Position) param<int>("position");
    bool        boxed       = param<bool>("boxed");

    int         basel_line  = 0;
    cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                         font_scale, thickness,
                                         &basel_line);

    const std::vector<int>& color_label = param<std::vector<int> >("color/label");
    const std::vector<int>& color_box   = param<std::vector<int> >("color/box");

    cv::Scalar cv_color_label(color_label[2], color_label[1], color_label[0]);
    cv::Scalar cv_color_box(color_box[2], color_box[1], color_box[0]);

    cv::Point  origin;
    switch(pos) {
    case TOP_LEFT:
        origin.x = 1;
        origin.y = text_size.height + 2;
        break;
    case BOTTOM_LEFT:
        origin.x = 1;
        origin.y = output->value.rows - 2 - basel_line;
        break;
    case TOP_RIGHT:
        origin.x = output->value.cols - text_size.width - 2;
        origin.y = text_size.height + 2;
        break;
    case BOTTOM_RIGHT:
        origin.x = output->value.cols - text_size.width - 2;
        origin.y = output->value.rows - 2 - basel_line;
        break;
    default:
        std::cerr << "Unkown alignment!" << std::endl;
        break;
    }

     if(boxed) {
        cv::Rect rect(origin.x - 2, origin.y - text_size.height,
                      text_size.width + 2, basel_line + text_size.height + 2);
        cv::rectangle(output->value,rect,cv_color_box,CV_FILLED);
    }

    cv::putText(output->value,
                label,
                origin,
                cv::FONT_HERSHEY_SIMPLEX,
                font_scale,
                cv_color_label,
                thickness,
                CV_AA);

    output_->publish(output);
}
