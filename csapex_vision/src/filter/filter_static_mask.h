#ifndef FILTER_STATIC_MASK_H
#define FILTER_STATIC_MASK_H

/// PROJECT
#include <csapex_opencv/filter.h>

namespace csapex
{
class FilterStaticMask : public Filter
{
    friend class StaticMaskSerializer;

public:
    FilterStaticMask();

public:
    void filter(cv::Mat& img, cv::Mat& mask) override;
    void setupParameters(Parameterizable& parameters) override;

    void setMask(const cv::Mat& m);
    cv::Mat getMask() const;

private:
    void showPainter();

public:
    slim_signal::Signal<void(cv::Mat&)> input;
    slim_signal::Signal<void()> show_painter;

private:
    cv::Mat mask_;
};

}  // namespace csapex

#endif  // FILTER_STATIC_MASK_H
