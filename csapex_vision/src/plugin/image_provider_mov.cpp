/// HEADER
#include "image_provider_mov.h"

/// PROJECT
#include <csapex/param/range_parameter.h>

/// SYSTEM
#include <boost/assign.hpp>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ImageProviderMov, csapex::MessageProvider)

using namespace csapex;

ImageProviderMov::ImageProviderMov()
{
}

void ImageProviderMov::load(const std::string& movie_file)
{
    capture_.open(movie_file);
    fps_ = capture_.get(CV_CAP_PROP_FPS);
    frames_ = capture_.get(CV_CAP_PROP_FRAME_COUNT);

    param::Parameter::Ptr p = state.getParameter("set/current_frame");
    param::RangeParameter::Ptr range_p = std::dynamic_pointer_cast<param::RangeParameter>(p);

    if(range_p) {
        range_p->setMax(frames_ - 1);
    }
}

ImageProviderMov::~ImageProviderMov()
{
    capture_.release();
}

std::vector<std::string> ImageProviderMov::getExtensions() const
{
    return boost::assign::list_of<std::string> (".avi")(".mpg")(".mp4")(".mov")(".flv");
}

bool ImageProviderMov::hasNext()
{
    if(!capture_.isOpened()) {
        return false;

    } else {
        return ImageProviderSet::hasNext();
    }
}

void ImageProviderMov::reallyNext(cv::Mat& img, cv::Mat& mask)
{
    if(!capture_.isOpened()) {
        std::cerr << "cannot display, capture not open" << std::endl;
        return;
    }

    int requested_frame = state.readParameter<int>("set/current_frame");
    bool skip = next_frame != requested_frame;

    if(next_frame >= frames_ && !skip) {
        img = last_frame_;
        setPlaying(false);
        return;
    }

    if(skip) {
        capture_.set(CV_CAP_PROP_POS_FRAMES, requested_frame);
    }

    capture_ >> last_frame_;

    img = last_frame_;

    next_frame = (int) capture_.get(CV_CAP_PROP_POS_FRAMES);
    state["set/current_frame"] = next_frame;

    if(next_frame == frames_) {
        bool loop = state.readParameter<bool>("set/loop");
        if(loop) {
            state["set/current_frame"] = 0;
        } else {
            setPlaying(false);
        }
    }
}
