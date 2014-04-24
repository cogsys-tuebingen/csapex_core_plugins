/// HEADER
#include "corner_line_detection.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CornerLineDetection::CornerLineDetection()
{
    Tag::createIfNotExists("Features");
    addTag(Tag::get("Features"));
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));
}

void CornerLineDetection::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Original");
    output_ = addOutput<CvMatMessage>("Corners / Lines / Edges");
}

