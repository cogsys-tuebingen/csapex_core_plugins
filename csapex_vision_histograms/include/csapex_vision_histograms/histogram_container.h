#ifndef HISTOGRAM_H
#define HISTOGRAM_H

/// SYSTEM
#include <cslibs_vision/utils/histogram.hpp>
#include <opencv2/opencv.hpp>

struct HistogramContainer
{
    std::vector<cslibs_vision::histogram::Rangef> ranges;
    std::vector<cv::Mat> histograms;
};

#endif  // HISTOGRAM_H
