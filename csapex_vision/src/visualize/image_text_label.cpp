/// HEADER
#include "image_text_label.h"

/// PROJECT
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::ImageTextLabel, csapex::Node)


ImageTextLabel::ImageTextLabel()
{
}

void ImageTextLabel::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("Image");
    output_ = node_modifier.addOutput<CvMatMessage>("Labeled");
}

void ImageTextLabel::setupParameters(Parameterizable& parameters)
{

    std::map<std::string, int> positions = {
        {"BOTTOM_LEFT", BOTTOM_LEFT},
        {"BOTTOM_RIGHT", BOTTOM_RIGHT},
        {"TOP_LEFT", TOP_LEFT},
        {"TOP_RIGHT", TOP_RIGHT}
    };
    parameters.addParameter(csapex::param::factory::declareParameterSet("position", positions, (int) TOP_LEFT));
    parameters.addParameter(csapex::param::factory::declareColorParameter("color/label", 255, 255, 255));
    parameters.addParameter(csapex::param::factory::declareColorParameter("color/box", 0, 0, 0));
    parameters.addParameter(csapex::param::factory::declareBool("boxed", false));
    parameters.addParameter(csapex::param::factory::declareRange("thickness", 1, 10, 1, 1));
    parameters.addParameter(csapex::param::factory::declareRange("scale", 1.0, 10.0, 1.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareText("label", "label"));

}

void ImageTextLabel::process()
{
    CvMatMessage::ConstPtr input  = msg::getMessage<CvMatMessage>(input_);
    if(input->value.type() != CV_8UC3 && input->value.type() != CV_8UC1)
        throw std::runtime_error("Must be mono or color image!");

    CvMatMessage::Ptr output(new CvMatMessage(input->getEncoding(), input->frame_id, input->stamp_micro_seconds));
    output->value = input->value.clone();

    std::string label       = readParameter<std::string>("label");
    double      font_scale  = readParameter<double>("scale");
    int         thickness   = readParameter<int>("thickness");
    Position    pos         = (Position) readParameter<int>("position");
    bool        boxed       = readParameter<bool>("boxed");

    int         basel_line  = 0;
    cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                         font_scale, thickness,
                                         &basel_line);

    const std::vector<int>& color_label = readParameter<std::vector<int> >("color/label");
    const std::vector<int>& color_box   = readParameter<std::vector<int> >("color/box");

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

    msg::publish(output_, output);
}
