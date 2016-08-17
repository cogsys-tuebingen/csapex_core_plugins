#ifndef IMAGE_PROVIDER_IMG_H
#define IMAGE_PROVIDER_IMG_H

/// COMPONENT
#include <csapex_opencv/image_provider.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{

class CSAPEX_EXPORT_PLUGIN ImageProviderImg : public ImageProvider
{
public:
    ImageProviderImg();
    void load(const std::string& file);


public:
    static std::function<bool(ImageProvider*)> Identity;
private:
    static bool checkIdentity(ImageProvider*);

public:
    bool hasNext();
    void next(cv::Mat& img, cv::Mat& mask);

    std::vector<std::string> getExtensions() const;

private:
    cv::Mat img_;

    bool sent_;
};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_IMG_H
