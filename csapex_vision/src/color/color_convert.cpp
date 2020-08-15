/// HEADER
#include "color_convert.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ColorConvert, csapex::Node)

using namespace csapex;
using namespace csapex;
using namespace csapex::connection_types;

ColorConvert::ColorConvert()
{
}

ColorConvert::~ColorConvert()
{
}

void ColorConvert::setup(NodeModifier& node_modifier)
{
    input_img_ = node_modifier.addInput<CvMatMessage>("original");
    output_img_ = node_modifier.addOutput<CvMatMessage>("converted");
}

void ColorConvert::setupParameters(Parameterizable& parameters)
{
    std::map<std::string, int> encodings = { { "YUV", (int)YUV }, { "RGB", (int)RGB },   { "BGR", (int)BGR }, { "HSL", (int)HSL },
                                             { "HSV", (int)HSV }, { "MONO", (int)MONO }, { "LAB", (int)LAB }, { "LUV", (int)LUV } };

    parameters.addParameter(csapex::param::factory::declareParameterSet("output", encodings, (int)MONO));
    parameters.addParameter(csapex::param::factory::declareParameterSet("input", encodings, (int)BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, BGR), (int)cv::COLOR_RGB2BGR));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, RGB), (int)cv::COLOR_BGR2RGB));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, YUV), (int)cv::COLOR_RGB2YUV));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, YUV), (int)cv::COLOR_BGR2YUV));
    cs_pair_to_operation_.insert(csiPair(csPair(YUV, RGB), (int)cv::COLOR_YUV2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(YUV, BGR), (int)cv::COLOR_YUV2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, HSV), (int)cv::COLOR_RGB2HSV));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, HSV), (int)cv::COLOR_BGR2HSV));
    cs_pair_to_operation_.insert(csiPair(csPair(HSV, RGB), (int)cv::COLOR_HSV2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(HSV, BGR), (int)cv::COLOR_HSV2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, HSL), (int)cv::COLOR_RGB2HLS));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, HSL), (int)cv::COLOR_BGR2HLS));
    cs_pair_to_operation_.insert(csiPair(csPair(HSL, RGB), (int)cv::COLOR_HLS2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(HSL, BGR), (int)cv::COLOR_HLS2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, MONO), (int)cv::COLOR_RGB2GRAY));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, MONO), (int)cv::COLOR_BGR2GRAY));
    cs_pair_to_operation_.insert(csiPair(csPair(MONO, RGB), (int)cv::COLOR_GRAY2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(MONO, BGR), (int)cv::COLOR_GRAY2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, LAB), (int)cv::COLOR_RGB2Lab));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, LAB), (int)cv::COLOR_BGR2Lab));
    cs_pair_to_operation_.insert(csiPair(csPair(LAB, RGB), (int)cv::COLOR_Lab2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(LAB, BGR), (int)cv::COLOR_Lab2BGR));

    cs_pair_to_operation_.insert(csiPair(csPair(RGB, LUV), (int)cv::COLOR_RGB2Luv));
    cs_pair_to_operation_.insert(csiPair(csPair(BGR, LUV), (int)cv::COLOR_BGR2Luv));
    cs_pair_to_operation_.insert(csiPair(csPair(LUV, RGB), (int)cv::COLOR_Luv2RGB));
    cs_pair_to_operation_.insert(csiPair(csPair(LUV, BGR), (int)cv::COLOR_Luv2BGR));

    cs_to_encoding_[YUV] = enc::yuv;
    cs_to_encoding_[RGB] = enc::rgb;
    cs_to_encoding_[BGR] = enc::bgr;
    cs_to_encoding_[HSL] = enc::hsl;
    cs_to_encoding_[HSV] = enc::hsv;
    cs_to_encoding_[MONO] = enc::mono;
    cs_to_encoding_[LAB] = enc::lab;
    cs_to_encoding_[LUV] = enc::luv;
}

void ColorConvert::process()
{
    CvMatMessage::ConstPtr img = msg::getMessage<CvMatMessage>(input_img_);

    csPair cspair;
    cspair.first = static_cast<ColorSpace>(readParameter<int>("input"));
    cspair.second = static_cast<ColorSpace>(readParameter<int>("output"));

    CvMatMessage::Ptr out(new CvMatMessage(cs_to_encoding_[cspair.second], img->frame_id, img->stamp_micro_seconds));

    if (img->getEncoding().channelCount() != cs_to_encoding_[cspair.first].channelCount()) {
        std::stringstream error;
        error << "Conversion not applicable! Input encoding #" << img->getEncoding().channelCount() << ", target #" << cs_to_encoding_[cspair.first].channelCount();
        throw std::runtime_error(error.str());
    }

    if (cspair.first != cspair.second) {
        if (cs_pair_to_operation_.find(cspair) != cs_pair_to_operation_.end()) {
            int mode = cs_pair_to_operation_[cspair];
            cv::cvtColor(img->value, out->value, mode);

            if ((int)out->getEncoding().channelCount() != out->value.channels()) {
                throw std::runtime_error("Conversion didn't work!");
            }
        } else {
            throw std::runtime_error("Conversion not supported!");
        }
        msg::publish(output_img_, out);

    } else {
        msg::publish(output_img_, img);
    }
}
