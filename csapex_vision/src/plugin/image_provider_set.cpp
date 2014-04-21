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
    state.addParameter(param::ParameterFactory::declareBool("set/playing", true));
    state.addParameter(param::ParameterFactory::declareRange("set/current_frame", 0, 1000, 0, 1));
    // TODO: interval param for playback borders
}

ImageProviderSet::~ImageProviderSet()
{
}

bool ImageProviderSet::hasNext()
{
    if(state.param<bool>("playback/resend")) {
        return true;
    }

    if(!state.param<bool>("set/playing")) {
        // not resend and not playing
        return false;
    }

    int requested_frame = state.param<int>("set/current_frame");
    return next_frame < frames_ || requested_frame < frames_;
}

void ImageProviderSet::next(cv::Mat& img, cv::Mat& mask)
{
    cv::Mat i;

    int requested_frame = state.param<int>("set/current_frame");
    if(state.param<bool>("set/playing") || requested_frame != next_frame) {
        reallyNext(i, mask);

    } else {
        i = last_frame_;
    }
    i.copyTo(img);
}

void ImageProviderSet::setPlaying(bool playing)
{
    state["set/playing"] = playing;
}
