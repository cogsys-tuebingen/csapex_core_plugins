/// HEADER
#include "filter_debayer.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Debayer, csapex::Node)

using namespace csapex;
using namespace csapex;
using namespace connection_types;

Debayer::Debayer()
{
}

void Debayer::setup(NodeModifier& node_modifier)
{
    input_img_ = node_modifier.addInput<CvMatMessage>("Image");
    output_img_ = node_modifier.addOutput<CvMatMessage>("Image");
}

void Debayer::setupParameters(Parameterizable& parameters)
{
    std::map<std::string, int> methods = {
        { "BayerBG2RGB", (int)CV_BayerBG2RGB }, { "BayerGB2RGB", (int)CV_BayerGB2RGB }, { "BayerRG2RGB", (int)CV_BayerRG2RGB }, { "BayerGR2RGB", (int)CV_BayerGR2RGB }, { "NNRG2RGB", 667 }
    };
    parameters.addParameter(csapex::param::factory::declareParameterSet("method", methods, (int)CV_BayerBG2RGB));
}

void Debayer::process()
{
    CvMatMessage::ConstPtr img_msg = msg::getMessage<CvMatMessage>(input_img_);
    const cv::Mat& img = img_msg->value;

    int mode = readParameter<int>("method");

    // assume 1 channel raw image comes in
    cv::Mat raw;
    if (img.channels() == 1) {
        raw = img;
    } else {
        ainfo << "first converting to gray image" << std::endl;
        cv::cvtColor(img, raw, CV_RGB2GRAY);
    }

    cv::Mat intermediate(raw.cols, raw.rows, CV_MAKE_TYPE(raw.depth(), 3));

    CvMatMessage::Ptr img_out(new CvMatMessage(enc::bgr, img_msg->frame_id, img_msg->stamp_micro_seconds));
    if (mode == 667) {
        debayerAndResize(raw, intermediate);
        cv::cvtColor(intermediate, intermediate, CV_BGR2RGB);
    } else {
        cv::cvtColor(raw, intermediate, mode);
    }

    img_out->value = intermediate;

    msg::publish(output_img_, img_out);
}

bool Debayer::usesMask()
{
    return false;
}

// Debayer: bayer-Pattern
// - every pixel has it's own color filter (e.g. only sees red)
// - pixel returns brightness value
void Debayer::debayerAndResize(const cv::Mat& source, cv::Mat& dest)
{
    cv::MatConstIterator_<uchar> it = source.begin<uchar>(), itEnd = source.end<uchar>();
    uchar* destination = (uchar*)dest.data;

    while (it != itEnd) {
        // r g r g r g
        // g b g b g b
        cv::MatConstIterator_<uchar> itLineEnd = it + 640;
        while (it != itLineEnd) {
            *destination = *it;
            ++it;
            ++destination;
            *destination = *it;
            ++it;
            ++destination;
            ++destination;
        }
        itLineEnd = it + 640;
        destination -= 320 * 3;
        while (it != itLineEnd) {
            // maybe add some green
            ++it;
            ++destination;
            ++destination;
            // add blue
            *destination = *it;
            ++it;
            ++destination;
        }
        destination += 320 * 3;
    }
}
