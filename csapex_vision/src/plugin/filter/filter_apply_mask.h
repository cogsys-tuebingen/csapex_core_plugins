#ifndef FILTER_APPLY_MASK_H
#define FILTER_APPLY_MASK_H

/// PROJECT
#include <csapex_opencv/filter.h>

namespace csapex
{
class FilterApplyMask : public Filter
{
public:
    FilterApplyMask();

    virtual void filter(cv::Mat& img, cv::Mat& mask);
};
}

#endif // FILTER_APPLY_MASK_H
