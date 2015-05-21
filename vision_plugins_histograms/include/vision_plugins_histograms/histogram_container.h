#ifndef HISTOGRAM_H
#define HISTOGRAM_H

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <utils_vision/utils/histogram.hpp>

struct HistogramContainer
{
    std::vector<utils_vision::histogram::Rangef> ranges;
    std::vector<cv::Mat>                        histograms;
};

#endif // HISTOGRAM_H
