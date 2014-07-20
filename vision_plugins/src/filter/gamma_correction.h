#ifndef GAMMA_CORRECTION_H
#define GAMMA_CORRECTION_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {


class GammaCorrection : public csapex::Node
{
    enum Type {
        POWER_LAW,
        LOGARITHM
    };

public:
    GammaCorrection();

    void setupParameters();
    void setup();
    void process();

private:
    ConnectorIn* in_;
    ConnectorOut* out_;
};


}

#endif // GAMMA_CORRECTION_H
