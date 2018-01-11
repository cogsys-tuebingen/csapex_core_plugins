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
using namespace csapex;

HOGChannels::HOGChannels()
{
}

void HOGChannels::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("bins", 1, 18, 9, 1),
                            bins_);
    parameters.addParameter(param::ParameterFactory::declareBool("signed", false),
                            signed_);
}

void HOGChannels::setup(NodeModifier& node_modifier)
{
    in_img_  = node_modifier.addInput<CvMatMessage>("image");
    out_img_ = node_modifier.addOutput<CvMatMessage>("hog channels");
}

void HOGChannels::process()
{
    CvMatMessage::ConstPtr  in = msg::getMessage<CvMatMessage>(in_img_);
    std::shared_ptr<std::vector<FeaturesMessage>> out(new std::vector<CvMatMessage>);

    const double bin_rad = M_PI / static_cast<double>(bins_);

    std::vector<cv::Mat> channels;
    if(signed_) {
        cslibs_vision::HOG::directed(in_img_->value, bin_rad, channels);
    } else {
        cslibs_vision::HOG::standard(in_img_->value, bin_rad, channels);
    }


    msg::publish<GenericVectorMessage, FeaturesMessage>(out_img_, out);
}
