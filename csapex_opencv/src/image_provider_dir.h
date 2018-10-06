#ifndef IMAGE_PROVIDER_DIR_H
#define IMAGE_PROVIDER_DIR_H

/// COMPONENT
#include "image_provider_img.h"
#include <csapex_opencv/image_provider.h>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN ImageProviderDir : public ImageProvider
{
public:
    ImageProviderDir();
    void load(const std::string& dir);

public:
    virtual ~ImageProviderDir();

public:
    static ImageProvider* createInstance(const std::string& path);

    virtual bool hasNext();
    void next(cv::Mat& img, cv::Mat& mask);

    std::vector<std::string> getExtensions() const;

private:
    bool is_right_format;
    const std::string dir_;

    cv::Mat img_, mask_;
    //    boost::filesystem::directory_iterator dir_it_;
    //    boost::filesystem::directory_iterator end_it_;
};

}  // namespace csapex

#endif  // IMAGE_PROVIDER_DIR_H
