#ifndef IMPORT_CIN_H
#define IMPORT_CIN_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class ImportCin : public Node
{
public:
    ImportCin();

    void process();
    virtual void setup();
    virtual void tick();

private:
    ConnectorOut* connector_;

    std::stringstream buffer;
};

}

#endif // IMPORT_CIN_H
