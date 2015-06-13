#ifndef BAR_CODE_READER_
#define BAR_CODE_READER_

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{

class BarCodeReader : public csapex::Node
{
public:
    BarCodeReader();

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& params) override;

    virtual void process() override;

private:
    Input* in_img;
    Output* out_str;
    Output* out_roi;

    std::string data_;
    bool lost;
    int forget;
};

}

#endif // BAR_CODE_READER_
