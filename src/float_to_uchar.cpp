#include "float_to_uchar.h"

/// HEADER
#include "flip.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/normalization.hpp>
#include <boost/assign.hpp>

using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(csapex::FloatToUchar, csapex::Node)

#warning "Temporary implementation fix with fixing the channel encoding!"

FloatToUchar::FloatToUchar() :
    type_(RELATIVE)
{
    addTag(Tag::get("Vision"));

    std::map<std::string, int> types = boost::assign::map_list_of
            ("RELATIVE", RELATIVE)
            ("ABSOLUTE", ABSOLUTE)
            ("SCALE",    SCALE);

    addParameter(param::ParameterFactory::declareParameterSet("type", types),
                 boost::bind(&FloatToUchar::update, this));
}

void FloatToUchar::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));
    out->value = in->value.clone();

    switch(type_) {
    case RELATIVE:
        utils_cv::normalizeRelative<uchar>(out->value);
        break;
    case ABSOLUTE:
        if((in->value.type() & 7) == CV_32F)
            utils_cv::normalizeAbsolute<float, uchar>(out->value);
        else
            utils_cv::normalizeAbsolute<double, uchar>(out->value);
        break;
    case SCALE:
            utils_cv::scale<uchar>(out->value);
        break;
    default:
        throw std::runtime_error("Cannot perfom this operation!");
    }


    out->value.convertTo(out->value, CV_8UC(in->value.channels()));
    output_->publish(out);
}

void FloatToUchar::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Float");
    output_ = addOutput<CvMatMessage>("Uchar");

    update();
}

void FloatToUchar::update()
{
    type_ = (Type) param<int>("type");
}
