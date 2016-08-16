#ifndef EXPORT_COUT_H
#define EXPORT_COUT_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN ExportCout : public Node
{
public:
    ExportCout();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* connector_;
};

}

#endif // EXPORT_COUT_H
