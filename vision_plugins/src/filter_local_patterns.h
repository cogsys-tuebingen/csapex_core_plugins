#ifndef FILTER_LOCAL_PATTERNS_H
#define FILTER_LOCAL_PATTERNS_H

/// COMPONENTS
#include <csapex_vision/filter.h>
#include <utils_cv/local_patterns.hpp>

namespace vision_plugins {

class LocalPatterns : public csapex::Filter
{
private:
    enum Type {
        LBP = 0,
        LTP = 1
    };

public:
    LocalPatterns();

    virtual void filter(cv::Mat &img, cv::Mat &mask);

private:
    utils_cv::LBP lbp_;
    utils_cv::LTP ltp_;

    std::vector<cv::Scalar> colors_;
};

}

#endif // FILTER_LOCAL_PATTERNS_H
