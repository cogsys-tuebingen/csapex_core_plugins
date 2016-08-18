#include "laplacian.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CSAPEX_REGISTER_CLASS(csapex::Laplacian, csapex::Node)

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
