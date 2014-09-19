#ifndef HISTOGRAM_H
#define HISTOGRAM_H

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <utils_vision/utils/histogram.hpp>

struct HistogramContainer
{
    utils_vision::histogram::Range  range;
    float                           bin_range;
    std::vector<cv::Mat>            histograms;
};

#endif // HISTOGRAM_H
