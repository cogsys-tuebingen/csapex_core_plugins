#ifndef IMAGE_PROVIDER_SET_H
#define IMAGE_PROVIDER_SET_H

/// COMPONENT
#include <csapex_opencv/image_provider.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN ImageProviderSet : public ImageProvider
{
protected:
    ImageProviderSet();
    virtual ~ImageProviderSet();

public:
    virtual void next(cv::Mat& img, cv::Mat& mask);

    void setPlaying(bool playing);

protected:  // abstract
    virtual void reallyNext(cv::Mat& img, cv::Mat& mask) = 0;

protected:
    virtual bool hasNext();

protected:
    cv::Mat last_frame_;

    double fps_;
    int frames_;
    int next_frame;
};

}  // namespace csapex

#endif  // IMAGE_PROVIDER_SET_H
