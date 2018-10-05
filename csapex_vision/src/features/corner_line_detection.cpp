/// HEADER
#include "corner_line_detection.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

CornerLineDetection::CornerLineDetection()
{
}

void CornerLineDetection::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("corners / lines / edges");
}
