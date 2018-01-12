/// HEADER
#include "hog_channels.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_ml/features_message.h>

#include <cslibs_vision/features/hog.hpp>

CSAPEX_REGISTER_CLASS(csapex::HOGChannels, csapex::Node)
/// TODO : L2HysThreshold - derivAperture

using namespace csapex;
using namespace csapex::connection_types;

HOGChannels::HOGChannels()
{
}

void HOGChannels::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("bins", 1, 18, 9, 1),
                            bins_);
    parameters.addParameter(param::ParameterFactory::declareRange("ksize", 1, 21, 3, 2),
                            ksize_);
    parameters.addParameter(param::ParameterFactory::declareBool("signed", false),
                            signed_);
}

void HOGChannels::setup(NodeModifier& node_modifier)
{
    in_img_  = node_modifier.addInput<CvMatMessage>("image");
    out_img_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage>("hog channels");
}

void HOGChannels::process()
{
    CvMatMessage::ConstPtr  in = msg::getMessage<CvMatMessage>(in_img_);
    std::shared_ptr<std::vector<CvMatMessage>> out(new std::vector<CvMatMessage>);

    const double bin_rad = M_PI / static_cast<double>(bins_);

    std::vector<cv::Mat> channels;
    if(signed_) {
        cslibs_vision::HOG::directed(in->value, bin_rad, channels, ksize_);
    } else {
        cslibs_vision::HOG::standard(in->value, bin_rad, channels, ksize_);
    }

    for(const cv::Mat &c : channels) {
        CvMatMessage m = CvMatMessage(enc::mono, in->frame_id, in->stamp_micro_seconds);
        m.value = c.clone();
        out->emplace_back(m);
    }

    msg::publish<GenericVectorMessage, CvMatMessage>(out_img_, out);
}
