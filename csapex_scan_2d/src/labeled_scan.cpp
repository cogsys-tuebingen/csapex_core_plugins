/// HEADER
#include <csapex_scan_2d/labeled_scan.h>

using namespace csapex;

void LabeledScan::init(Scan &other)
{
    Scan::operator= (other);

    labels.resize(ranges.size(), 0);
}
