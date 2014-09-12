#include "scharr.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/normalization.hpp>
#include <boost/assign.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Scharr, csapex::Node)

Scharr::Scharr()  :
    type_(DX1)
{
}

void Scharr::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono));
    int depth = in->value.type() & 7;
    switch(type_) {
    case DX1:
        cv::Scharr(in->value, out->value, depth,1, 0, ksize_, scale_,delta_);
        break;
    case DY1:
        cv::Scharr(in->value, out->value, depth,0, 1, ksize_, scale_,delta_);
        break;
    case STRENGTH:
    {
        cv::Mat grad_x, grad_y (in->value.rows,in->value.cols,CV_32F);
        cv::Mat abs_grad_x, abs_grad_y (in->value.rows,in->value.cols,CV_8U);
        cv::Scharr(in->value, grad_x, depth,1, 0, ksize_, scale_,delta_);
        cv::Scharr(in->value, grad_y, depth,0, 1, ksize_, scale_,delta_);
        cv::convertScaleAbs( grad_x, abs_grad_x );
        cv::convertScaleAbs( grad_y, abs_grad_y );
        cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, out->value);
        break;
    }
    default:
        throw std::runtime_error("Unknown derivation type!");
    }
    output_->publish(out);
}

void Scharr::setupParameters()
{
    Operator::setupParameters();
    std::map<std::string, int> types = boost::assign::map_list_of
            ("DX1", DX1)
            ("DY1", DY1)
            ("Strength", STRENGTH);

    addParameter(param::ParameterFactory::declareParameterSet("derive", types, (int) DX1),
                 boost::bind(&Scharr::update, this));
}

void Scharr::update()
{
    Operator::update();
    type_ = (Type) readParameter<int>("derive");
}
