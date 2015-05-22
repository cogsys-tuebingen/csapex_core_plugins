/// HEADER
#include "filter_debayer.h"

/// PROJECT
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <QComboBox>
#include <QLabel>
#include <boost/assign/list_of.hpp>



CSAPEX_REGISTER_CLASS(vision_plugins::Debayer, csapex::Node)

using namespace vision_plugins;
using namespace csapex;
using namespace connection_types;

Debayer::Debayer()
{
}

void Debayer::setup(NodeModifier &node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("Image");
    output_ = node_modifier.addOutput<CvMatMessage>("Debayered Image");
}

void Debayer::setupParameters(Parameterizable& parameters)
{
    std::map<std::string, int> methods = boost::assign::map_list_of
            ("BayerBG2RGB", (int) CV_BayerBG2RGB)
            ("BayerGB2RGB", (int) CV_BayerGB2RGB)
            ("BayerRG2RGB", (int) CV_BayerRG2RGB)
            ("BayerGR2RGB", (int) CV_BayerGR2RGB)
            ("NNRG2RGB", 667);

    parameters.addParameter(param::ParameterFactory::declareParameterSet("method", methods, (int) CV_BayerBG2RGB));
}

void Debayer::process()
{
    CvMatMessage::ConstPtr img_msg = msg::getMessage<CvMatMessage>(input_);
    cv::Mat img = img_msg->value.clone();
    int mode = readParameter<int>("method");

    // assume 1 channel raw image comes in
    cv::Mat raw;
    if(img.channels() == 1) {
        raw = img;
    } else {
        cv::cvtColor(img, raw, CV_RGB2GRAY);
    }
    if (mode == 667) {
        this->debayerAndResize(raw, img);
        cv::cvtColor(img, img, CV_BGR2RGB);
    }
    else {
        cv::cvtColor(raw, img, mode);
    }
    CvMatMessage::Ptr result(new CvMatMessage(enc::bgr,img_msg->stamp_micro_seconds));
    result->value = img;
    msg::publish(output_,result);
}

bool Debayer::usesMask()
{
    return false;
}

// Debayer: bayer-Pattern
// - every pixel has it's own color filter (e.g. only sees red)
// - pixel returns brightness value
void Debayer::debayerAndResize(cv::Mat& source, cv::Mat& dest) {

    cv::MatIterator_<uchar> it = source.begin<uchar>(),
                         itEnd = source.end<uchar>();
    uchar* destination = (uchar*) dest.data;

    while(it != itEnd) {
        // r g r g r g
        // g b g b g b
        cv::MatIterator_<uchar> itLineEnd = it + 640;
        while(it != itLineEnd) {
            *destination = *it;
            ++it;
            ++destination;
            *destination = *it;
            ++it;
            ++destination;
            ++destination;
        }
        itLineEnd = it + 640;
        destination -= 320*3;
        while(it != itLineEnd) {
            // maybe add some green
            ++it;
            ++destination;
            ++destination;
            // add blue
            *destination = *it;
            ++it;
            ++destination;
        }
        destination += 320*3;
    }
}

