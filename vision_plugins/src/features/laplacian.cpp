#include "laplacian.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <boost/assign.hpp>
#include <utils_cv/normalization.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Laplacian, csapex::Node)

Laplacian::Laplacian()
{
}


void Laplacian::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->stamp));

    if(in->value.empty())
        return;

    int depth = in->value.type() & 7;
    cv::Laplacian(in->value, out->value, depth, ksize_, scale_, delta_);

    output_->publish(out);
}
