#ifndef HOG_EXTRACTOR_H
#define HOG_EXTRACTOR_H

/// PROJECT
#include <csapex/model/node.h>

namespace vision_plugins {
class HOGExtractor : public csapex::Node
{
public:
    HOGExtractor();

    void setupParameters();
    void setup();
    void process();

private:
    const static int HOG_BLOCK_WIDTH  = 16; /// ONLY SUPPORT BY NOW
    const static int HOG_BLOCK_HEIGHT = 16; /// ONLY SUPPORT BY NOW
    const static int HOG_CELL_WIDTH   = 8;  /// ONLY SUPPORT BY NOW
    const static int HOG_CELL_HEIGHT  = 8;  /// ONLY SUPPORT BY NOW

    csapex::ConnectorIn  *in_;
    csapex::ConnectorOut *out_;

};
}
#endif // HOG_EXTRACTOR_H
