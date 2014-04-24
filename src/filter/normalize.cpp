/// HEADER
#include "normalize.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/assign/list_of.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Normalize, csapex::Node)

Normalize::Normalize()
{
    addTag(Tag::get("Filter"));
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));;
    std::map<std::string, int> norms = boost::assign::map_list_of
            ("L2",     (int) cv::NORM_L2)
            ("L1",     (int) cv::NORM_L1)
            ("INF",    (int) cv::NORM_INF)
            ("MINMAX", (int) cv::NORM_MINMAX);
    addParameter(param::ParameterFactory::declareParameterSet("norm", norms));
    addParameter(param::ParameterFactory::declareRange("lower bound scale", -255.0, 255.0, 1.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("upper bound scale", -255.0, 255.0, 0.0, 0.1));
}

void Normalize::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr mask;
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    if(mask_->isConnected()) {
        mask = mask_->getMessage<CvMatMessage>();
    }

    int     norm  = param<int>("norm");
    double  lower = param<double>("lower bound scale");
    double  upper = param<double>("upper bound scale");

    cv::normalize(in->value,out->value, lower, upper, norm, -1,
                  mask.get() == NULL ? cv::noArray() : mask->value);

    output_->publish(out);
}

void Normalize::setup()
{
    setSynchronizedInputs(true);

    input_  = addInput<CvMatMessage>("original");
    mask_   = addInput<CvMatMessage>("mask", true);
    output_ = addOutput<CvMatMessage>("normalized");
}
