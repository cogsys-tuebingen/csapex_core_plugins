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

    std::vector<std::vector<double> > median_matrix_;

private:
    ConnectorIn* input_transform_;
    ConnectorOut* output_transform_;
    ConnectorOut* output_text_; // debug output for text

    void tfToXYZrpy(tf::Transform in, double& x, double& y, double& z, double& roll, double& pitch, double& yaw);
    void runFilter(tf::Transform in_new, tf::Transform out);

    double mean(std::vector<double> in);

};

}
#endif // TRANSFORM_FILTER_H
