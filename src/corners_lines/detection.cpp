/// HEADER
#include "detection.h"

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

CornerLineDetection::CornerLineDetection()
{
    Tag::createIfNotExists("Corners");
    Tag::createIfNotExists("Lines");
    addTag(Tag::get("Corners"));
    addTag(Tag::get("Vision"));
}

void CornerLineDetection::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Original");
    output_ = addOutput<CvMatMessage>("Corners / Lines");
}

