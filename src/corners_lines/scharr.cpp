#include "scharr.h"

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

CSAPEX_REGISTER_CLASS(csapex::Scharr, csapex::Node)

Scharr::Scharr()  :
    type_(DX1)
{
    std::map<std::string, int> types = boost::assign::map_list_of
            ("DX1", DX1)
            ("DY1", DY1);

    addParameter(param::ParameterFactory::declareParameterSet("derive", types),
                 boost::bind(&Scharr::update, this));
}

void Scharr::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono));
    switch(type_) {
    case DX1:
        cv::Scharr(in->value, out->value, ddepth_,1, 0, ksize_, scale_,delta_);
        break;
    case DY1:
        cv::Scharr(in->value, out->value, ddepth_,0, 1, ksize_, scale_,delta_);
        break;
    default:
        throw std::runtime_error("Unknown derivation type!");
    }
    output_->publish(out);
}

void Scharr::update()
{
    Operator::update();
    type_ = (Type) param<int>("derive");
}
