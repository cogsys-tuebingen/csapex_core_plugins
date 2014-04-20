#ifndef IMAGE_PROVIDER_SET_H
#define IMAGE_PROVIDER_SET_H

/// COMPONENT
#include <csapex_vision/image_provider.h>

namespace csapex
{

class ImageProviderSet : public ImageProvider
{
protected:
    ImageProviderSet();
    virtual ~ImageProviderSet();

public:
    virtual void next(cv::Mat& img, cv::Mat& mask);

    void setPlaying(bool playing);

protected: // abstract
    virtual void reallyNext(cv::Mat& img, cv::Mat& mask) = 0;

protected:
    cv::Mat last_frame_;

    double fps_;
    int frames_;
    int next_frame;
};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_SET_H
