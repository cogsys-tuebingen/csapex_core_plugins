/// HEADER
#include "gradient_histogram.h"


/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_opencv/cv_mat_message.h>

#include <cslibs_vision/features/gradient_histogram.hpp>

CSAPEX_REGISTER_CLASS(csapex::GradientHistogram, csapex::Node)
/// TODO : L2HysThreshold - derivAperture

using namespace csapex;
using namespace csapex::connection_types;

GradientHistogram::GradientHistogram()
{
}

void GradientHistogram::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareAngle("min_angle", 0.0),
                            interval_[0]);
    parameters.addParameter(param::ParameterFactory::declareAngle("max_angle", M_PI / 4.0),
                            interval_[1]);

    parameters.addParameter(param::ParameterFactory::declareRange("ksize", 1, 21, 3, 2),
                            ksize_);
    parameters.addParameter(param::ParameterFactory::declareBool("signed", false),
                            signed_);
}

void GradientHistogram::setup(NodeModifier& node_modifier)
{
    in_img_   = node_modifier.addInput<CvMatMessage>("image");
    out_hist_ = node_modifier.addOutput<CvMatMessage>("histogram");
    out_mag_  = node_modifier.addOutput<CvMatMessage>("magnitude");
}

void GradientHistogram::process()
{
    CvMatMessage::ConstPtr  in  = msg::getMessage<CvMatMessage>(in_img_);
    CvMatMessage::Ptr       hist(new CvMatMessage(enc::mono, in->frame_id, in->stamp_micro_seconds));
    CvMatMessage::Ptr       mag(new CvMatMessage(enc::unknown, in->frame_id, in->stamp_micro_seconds));

    if(signed_) {
        cslibs_vision::GradientHistogram::directed(in->value, interval_, hist->value, mag->value, ksize_);
    } else {
        cslibs_vision::GradientHistogram::standard(in->value, interval_, hist->value, mag->value, ksize_);
    }

    msg::publish(out_hist_, hist);
    msg::publish(out_mag_, mag);
}
