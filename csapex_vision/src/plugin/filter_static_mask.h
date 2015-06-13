#ifndef FILTER_STATIC_MASK_H
#define FILTER_STATIC_MASK_H

/// PROJECT
#include <csapex_vision/filter.h>

namespace csapex
{

class FilterStaticMask : public Filter
{
    friend class StaticMaskSerializer;

public:
    FilterStaticMask();

public:
    virtual void filter(cv::Mat& img, cv::Mat& mask) override;
    virtual void setupParameters(Parameterizable &parameters) override;

    void setMask(const cv::Mat& m);
    cv::Mat getMask() const;

private:
    void showPainter();

public:
    boost::signals2::signal<void(cv::Mat&)> input;
    boost::signals2::signal<void()> show_painter;

private:
    cv::Mat mask_;
};

} /// NAMESPACE

#endif // FILTER_STATIC_MASK_H
