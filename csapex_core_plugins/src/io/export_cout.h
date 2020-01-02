#ifndef EXPORT_COUT_H
#define EXPORT_COUT_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN ExportCout : public Node
{
public:
    ExportCout();

    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    Input* connector_;
};

}  // namespace csapex

#endif  // EXPORT_COUT_H
