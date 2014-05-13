#ifndef HISTOGRAM_MAXIMA_CONTAINER_H
#define HISTOGRAM_MAXIMA_CONTAINER_H

#include <vector>
struct HistogramMaximaContainer {
    typedef std::pair<unsigned int, float> Maximum;

    /// first  == bin the maximum is related to
    /// second == the value the maximum has

    std::vector<std::vector<Maximum> > maxima;
};


#endif // HISTOGRAM_MAXIMA_CONTAINER_H
