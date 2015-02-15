/// HEADER
#include "corner_line_detection.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CornerLineDetection::CornerLineDetection()
{
}

void CornerLineDetection::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("original");
    output_ = modifier_->addOutput<CvMatMessage>("corners / lines / edges");
}

