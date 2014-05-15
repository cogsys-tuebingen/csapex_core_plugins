#ifndef HISTOGRAM_MAXIMA_CONTAINER_H
#define HISTOGRAM_MAXIMA_CONTAINER_H

#include <vector>
struct HistogramMaximaContainer {
    typedef std::pair<unsigned int, float> Maximum;
    /// position in the histogram
    std::vector<std::vector<Maximum> > maxima;
    float                              bin_range;
};


#endif // HISTOGRAM_MAXIMA_CONTAINER_H
