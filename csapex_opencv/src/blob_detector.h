#ifndef BLOB_DETECTOR_H
#define BLOB_DETECTOR_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN BlobDetector : public csapex::Node
{
public:
    BlobDetector();
    ~BlobDetector();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Output* output_;
    Output* output_debug_;
    Output* output_reduce_;

    Input* input_;
};
}  // namespace csapex

#endif  // BLOB_DETECTOR_H
