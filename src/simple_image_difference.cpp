/// HEADER
#include "simple_image_difference.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

using namespace vision_plugins;
using namespace csapex;
using namespace connection_types;

CSAPEX_REGISTER_CLASS(vision_plugins::SimpleImageDifference, csapex::Node)

SimpleImageDifference::SimpleImageDifference()
{
}

void SimpleImageDifference::allConnectorsArrived()
{
    CvMatMessage::Ptr img1 = in_a_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr img2 = in_b_->getMessage<CvMatMessage>();

    CvMatMessage::Ptr out(new CvMatMessage);
    cv::absdiff(img1->value, img2->value, out->value);

    out_->publish(out);
}

void SimpleImageDifference::setup()
{
    setSynchronizedInputs(true);

    in_a_ = addInput<CvMatMessage>("A");
    in_b_ = addInput<CvMatMessage>("B");

    out_ = addOutput<CvMatMessage>("A-B");
}
