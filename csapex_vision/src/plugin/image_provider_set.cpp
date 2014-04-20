/// HEADER
#include "image_provider_set.h"

/// PROJECT
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <QHBoxLayout>

using namespace csapex;

ImageProviderSet::ImageProviderSet()
    : next_frame(-1)
{
    state.addParameter(param::ParameterFactory::declareBool("playing", true));
    state.addParameter(param::ParameterFactory::declareRange("current_frame", 0, 1000, 0, 1));
    // TODO: interval param for playback borders
}

ImageProviderSet::~ImageProviderSet()
{
}

void ImageProviderSet::next(cv::Mat& img, cv::Mat& mask)
{
    cv::Mat i;
    if(state.param<bool>("playing")) {
        reallyNext(i, mask);

    } else {
        i = last_frame_;
    }
    i.copyTo(img);
}

void ImageProviderSet::setPlaying(bool playing)
{
    state["playing"] = playing;
}
