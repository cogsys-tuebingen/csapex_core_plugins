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
    Input* input_transform_;
    Output* output_transform_;
    Output* output_text_; // debug output for text

    unsigned int filter_size_;
    unsigned int filter_index_;

    std::vector<double> out_vector_latch_;

    void tfToXYZrpy(tf::Transform& in, double& x, double& y, double& z, double& roll, double& pitch, double& yaw);
    void runFilter(tf::Transform &in_new, tf::Transform &out);

    double mean(std::vector<double> in);

};

}
#endif // TRANSFORM_FILTER_H
