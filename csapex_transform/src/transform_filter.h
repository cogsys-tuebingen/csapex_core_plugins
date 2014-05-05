#ifndef TRANSFORM_FILTER_H
#define TRANSFORM_FILTER_H

/// PROJECT
#include <csapex/model/node.h>
#include <tf/tf.h>

namespace csapex {

class TransformFilter : public csapex::Node
{
public:
    TransformFilter();

    virtual void setup();
    virtual void process();

private:
    ConnectorIn* input_transform_;
    ConnectorOut* output_transform_;
    ConnectorOut* output_text_; // debug output for text

    void tfToXYZrpy(tf::Transform in, double& x, double& y, double& z, double& roll, double& pitch, double& yaw);
    void runFilter(tf::Transform in_new, tf::Transform out);

};

}
#endif // TRANSFORM_FILTER_H
