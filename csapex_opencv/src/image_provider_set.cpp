/// HEADER
#include "image_provider_set.h"

/// PROJECT
#include <csapex/param/parameter_factory.h>

using namespace csapex;

ImageProviderSet::ImageProviderSet() : next_frame(-1)
{
    state.addParameter(csapex::param::factory::declareBool("set/playing", true));
    state.addParameter(csapex::param::factory::declareBool("set/loop", false));
    state.addParameter(csapex::param::factory::declareRange("set/current_frame", 0, 1000, 0, 1));
    // TODO: interval param for playback borders
}

ImageProviderSet::~ImageProviderSet()
{
}

bool ImageProviderSet::hasNext()
{
    if (state.readParameter<bool>("playback/resend")) {
        return true;
    }

    if (!state.readParameter<bool>("set/playing")) {
        // not resend and not playing
        return false;
    }

    if (state.readParameter<bool>("set/loop")) {
        return true;
    }

    int requested_frame = state.readParameter<int>("set/current_frame");
    return next_frame < frames_ || requested_frame < frames_;
}

void ImageProviderSet::next(cv::Mat& img, cv::Mat& mask)
{
    cv::Mat i;

    int requested_frame = state.readParameter<int>("set/current_frame");
    if (state.readParameter<bool>("set/playing") || requested_frame != next_frame) {
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
