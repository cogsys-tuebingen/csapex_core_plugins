/// HEADER
#include "normalize.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/assign/list_of.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Normalize, csapex::Node)

Normalize::Normalize()
{
}

void Normalize::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr mask;
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp));

    if(mask_->hasMessage()) {
        mask = mask_->getMessage<CvMatMessage>();
    }

    int     norm  = readParameter<int>("norm");
    double  lower = readParameter<double>("lower bound scale");
    double  upper = readParameter<double>("upper bound scale");

    in->value.copyTo(out->value);
    cv::normalize(in->value, out->value, lower, upper, norm, -1,
                  mask.get() == NULL ? cv::noArray() : mask->value);

    output_->publish(out);
}

void Normalize::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("original");
    mask_   = modifier_->addOptionalInput<CvMatMessage>("mask");
    output_ = modifier_->addOutput<CvMatMessage>("normalized");
}

void Normalize::setupParameters()
{
    std::map<std::string, int> norms = boost::assign::map_list_of
            ("L2",     (int) cv::NORM_L2)
            ("L1",     (int) cv::NORM_L1)
            ("INF",    (int) cv::NORM_INF)
            ("MINMAX", (int) cv::NORM_MINMAX);
    addParameter(param::ParameterFactory::declareParameterSet("norm", norms, (int) cv::NORM_L2));
    addParameter(param::ParameterFactory::declareRange("lower bound scale", -255.0, 255.0, 1.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("upper bound scale", -255.0, 255.0, 0.0, 0.1));
}
