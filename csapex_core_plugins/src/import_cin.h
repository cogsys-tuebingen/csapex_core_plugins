#ifndef IMPORT_CIN_H
#define IMPORT_CIN_H

/// PROJECT
#include <csapex/model/tickable_node.h>

namespace csapex {

class ImportCin : public TickableNode
{
public:
    ImportCin();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void tick() override;

private:
    Output* connector_;

    std::stringstream buffer;
};

}

#endif // IMPORT_CIN_H
