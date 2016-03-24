#ifndef HOGCLASSIFIER_H
#define HOGCLASSIFIER_H

#include "hog.h"

/// https://github.com/DaHoC/trainHOG/wiki/trainHOG-Tutorial
/// TO CLASSIFY ROI DIRECTLY

namespace vision_plugins {
class HOGClassifier
{
public:
    HOGClassifier();

private:
    std::size_t cells_x_;
    std::size_t cells_y_;
    std::size_t cell_size_;
    std::size_t block_size_;

    HOGDescriptor hog_;

};
}
#endif // HOGCLASSIFIER_H
