/// HEADER
#include "gamma_correction.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::GammaCorrection, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


GammaCorrection::GammaCorrection()
{
}

void GammaCorrection::setupParameters()
{
    std::map<std::string, int> types = boost::assign::map_list_of
            ("power law", (int) POWER_LAW)
            ("logarithm", (int) LOGARITHM);

    param::Parameter::Ptr type = param::ParameterFactory::declareParameterSet<int>("type",
                                                                                   param::ParameterDescription("The type of transformation to apply"),
                                                                                   types, POWER_LAW);
    addParameter(type);

    addParameter(param::ParameterFactory::declareRange("c",
                                                       param::ParameterDescription("Constant factor in  dst = c * log(src + 1)"),
                                                       0.1, 255.0, 1.0, 0.01));

    addConditionalParameter(param::ParameterFactory::declareRange("gamma",
                                                                  param::ParameterDescription("Constant factor in  dst = c * (src ^ gamma)"),
                                                                  0.001, 10.0, 1.0, 0.001),
                            (boost::bind(&param::Parameter::as<int>, type.get()) == POWER_LAW));
}

void GammaCorrection::setup()
{
    in_  = modifier_->addInput<CvMatMessage>("Image");
    out_ = modifier_->addOutput<CvMatMessage>("Image");
}

void GammaCorrection::process()
{
    CvMatMessage::ConstPtr input = in_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr output(new CvMatMessage(input->getEncoding(), input->stamp));

    const cv::Mat& in = input->value;
    cv::Mat& out = output->value;

    cv::Mat tmp;
    in.convertTo(tmp, CV_32F);


    double c = readParameter<double>("c");

    switch(readParameter<int>("type")) {
    case POWER_LAW:
        tmp /= 255.0;
        cv::pow(tmp, readParameter<double>("gamma"), tmp);
        tmp *= c * 255.0;
        break;

    case LOGARITHM:
        tmp += 1;
        cv::log(tmp, tmp);
        tmp *= c;
        break;

    default:
        break;
    }


    tmp.convertTo(out, in.type());

    out_->publish(output);
}


