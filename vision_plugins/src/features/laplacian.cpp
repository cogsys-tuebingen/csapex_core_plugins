#include "laplacian.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <boost/assign.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Laplacian, csapex::Node)

Laplacian::Laplacian()
{
}


void Laplacian::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->stamp_micro_seconds));

    if(in->value.empty())
        return;

    int depth = in->value.type() & 7;
    cv::Laplacian(in->value, out->value, depth, ksize_, scale_, delta_);

    msg::publish(output_, out);
}
