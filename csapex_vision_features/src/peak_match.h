#ifndef IMAGE_COMBINER_ROBUST_MATCH_H
#define IMAGE_COMBINER_ROBUST_MATCH_H
/// COMPONENT
#include <csapex/model/node.h>


namespace csapex
{

class PeakMatch : public csapex::Node
{
public:
    PeakMatch();

    virtual void setup();
    virtual void process();

private:
    ConnectorIn* in_img_1;
    ConnectorIn* in_key_1;
    ConnectorIn* in_des_1;

    ConnectorIn* in_img_2;
    ConnectorIn* in_key_2;
    ConnectorIn* in_des_2;

    ConnectorOut* out_img;
};

} /// NAMESPACE

#endif // IMAGE_COMBINER_ROBUST_MATCH_H
