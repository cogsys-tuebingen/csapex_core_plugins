#ifndef HISTOGRAM_H
#define HISTOGRAM_H

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <utils_cv/histogram.hpp>

struct Histograms
{
    utils_cv::Range      range;
    std::vector<cv::Mat> histograms;
};

#endif // HISTOGRAM_H
