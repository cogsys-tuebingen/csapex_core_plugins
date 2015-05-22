/// HEADER
#include "image_provider_img.h"

/// SYSTEM
#include <boost/assign.hpp>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ImageProviderImg, csapex::MessageProvider)


using namespace csapex;


std::function<bool(ImageProvider*)> ImageProviderImg::Identity
= std::bind(&ImageProviderImg::checkIdentity, std::placeholders::_1);

ImageProviderImg::ImageProviderImg()
    : sent_(false)
{
}

void ImageProviderImg::load(const std::string& path)
{
    img_ = cv::imread(path, CV_LOAD_IMAGE_UNCHANGED);
    sent_ = false;
}

std::vector<std::string> ImageProviderImg::getExtensions() const
{
    return boost::assign::list_of<std::string> (".jpg")(".jpeg")(".gif")(".png")(".tiff")(".pgm")(".ppm");
}

bool ImageProviderImg::checkIdentity(ImageProvider* other)
{
    return dynamic_cast<ImageProviderImg*>(other) != nullptr;
}

void ImageProviderImg::next(cv::Mat& img, cv::Mat& mask)
{
    img_.copyTo(img);

    sent_ = true;
}

bool ImageProviderImg::hasNext()
{
    return !sent_ || state.readParameter<bool>("playback/resend");
}
