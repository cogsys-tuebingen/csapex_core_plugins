/// HEADER
#include "filter_equalize.h"

/// PROJECT
#include <utils/LibCvTools/histogram.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(vision_plugins::Equalize, csapex::Node)

using namespace vision_plugins;

Equalize::Equalize()
{
}

Equalize::~Equalize()
{
}

void Equalize::insert(QBoxLayout *parent)
{
}

void Equalize::filter(cv::Mat &img, cv::Mat &mask)
{
    cv_histogram::full_channel_equalize(img, img);
}

bool Equalize::usesMask()
{
    return false;
}
