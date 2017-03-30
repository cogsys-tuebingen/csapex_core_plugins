/// HEADER
#include "normalize.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <opencv2/imgproc/imgproc.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::Normalize, csapex::Node)

Normalize::Normalize()
{
}

void Normalize::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::ConstPtr mask;
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));

    if(msg::hasMessage(mask_)) {
        mask = msg::getMessage<CvMatMessage>(mask_);
    }

    int     norm  = readParameter<int>("norm");
    double  lower = readParameter<double>("lower bound scale");
    double  upper = readParameter<double>("upper bound scale");

    in->value.copyTo(out->value);
    cv::normalize(in->value, out->value, lower, upper, norm, -1,
                  mask.get() == nullptr ? cv::noArray() : mask->value);

    msg::publish(output_, out);
}

void Normalize::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("original");
    mask_   = node_modifier.addOptionalInput<CvMatMessage>("mask");
    output_ = node_modifier.addOutput<CvMatMessage>("normalized");
}

void Normalize::setupParameters(Parameterizable& parameters)
{
    std::map<std::string, int> norms = {
        {"L2",     (int) cv::NORM_L2},
        {"L1",     (int) cv::NORM_L1},
        {"INF",    (int) cv::NORM_INF},
        {"MINMAX", (int) cv::NORM_MINMAX}
    };
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterSet("norm", norms, (int) cv::NORM_L2));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("lower bound scale", -255.0, 255.0, 1.0, 0.1));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("upper bound scale", -255.0, 255.0, 0.0, 0.1));
}
