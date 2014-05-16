#ifndef SCAN_H
#define SCAN_H

/// SYSTEM
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

namespace csapex
{

struct Scan
{
public:
    typedef boost::shared_ptr<Scan> Ptr;

public:
    struct Header {
        unsigned int seq;

        unsigned long stamp_nsec;

        std::string frame_id;
    };

    Header header;

    float angle_min;
    float angle_max;

    float angle_increment;

    float range_min;

    float range_max;

    std::vector<float> ranges;

};

}

#endif // SCAN_H
