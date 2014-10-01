#include "convert_type.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/assign/std.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::ConvertType, csapex::Node)

namespace {
    template<typename Tp>
    inline void normalize(cv::Mat &mat)
    {
        double min;
        double max;
        cv::minMaxLoc(mat, &min, &max);
        mat = mat - min;
        max -= min;
        mat = mat / max;
        Tp fac = std::numeric_limits<Tp>::max();
        mat = mat * fac;
    }
}

ConvertType::ConvertType() :
    mode_(CV_8U)
{
}

void ConvertType::process()
{
#warning "Change to csapex type encoding!"
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp));
    out->value = in->value.clone();

    switch (mode_) {
    case CV_8U:
        if(normalize_)
            normalize<unsigned char>(out->value);
        out->value.convertTo(out->value,  CV_8UC(in->value.channels()));
        break;
    case CV_8S:
        if(normalize_)
            normalize<signed char>(out->value);
        out->value.convertTo(out->value,  CV_8SC(in->value.channels()));
        break;
    case CV_16U:
        if(normalize_)
            normalize<unsigned short int>(out->value);
        out->value.convertTo(out->value, CV_16UC(in->value.channels()));
        break;
    case CV_16S:
        if(normalize_)
            normalize<signed short int>(out->value);
        out->value.convertTo(out->value, CV_16SC(in->value.channels()));
        break;
    case CV_32S:
        if(normalize_)
            normalize<signed int>(out->value);
        out->value.convertTo(out->value, CV_32SC(in->value.channels()));
        break;
    case CV_32F:
        if(normalize_)
            normalize<float>(out->value);
        out->value.convertTo(out->value, CV_32FC(in->value.channels()));
        break;
    case CV_64F:
        if(normalize_)
            normalize<double>(out->value);
        out->value.convertTo(out->value, CV_64FC(in->value.channels()));
        break;
    default:
        throw std::runtime_error("Unknown conversion goal type!");
        break;
    }

    output_->publish(out);
}

void ConvertType::setup()
{
    input_ =  modifier_->addInput<CvMatMessage>("original");
    output_ = modifier_->addOutput<CvMatMessage>("converted");
    update();
}

void ConvertType::setupParameters()
{
    std::map<std::string, int> types = boost::assign::map_list_of
            (" 8 Bit unsigned", CV_8U)
            (" 8 Bit signed",   CV_8S)
            ("16 Bit usigned",  CV_16U)
            ("16 Bit signed",   CV_16S)
            ("32 Bit signed",   CV_32S)
            ("32 Bit floating", CV_32F)
            ("64 Bit floating", CV_64F);

    addParameter(param::ParameterFactory::declareParameterSet("convert to", types, CV_8U),
                 boost::bind(&ConvertType::update, this));

    addParameter(param::ParameterFactory::declareBool("normalize", false),
                 boost::bind(&ConvertType::update, this));
}

void ConvertType::update()
{
    mode_ = readParameter<int>("convert to");
}
