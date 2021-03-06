/// HEADER
#include "transform_filter.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

#define RAD_TO_DEG(x) ((x))
// #define RAD_TO_DEG(x) ((x) * 57.29578)

CSAPEX_REGISTER_CLASS(csapex::TransformFilter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformFilter::TransformFilter()
{
    median_matrix_.resize(6);

    // Initialize the out_vector_latch with zeros
    for (int i = 0; i < 6; i++) {
        out_vector_latch_.push_back(0);
    }
}

void TransformFilter::setup(NodeModifier& node_modifier)
{
    input_transform_ = node_modifier.addInput<connection_types::TransformMessage>("Raw Transformation");

    output_transform_ = node_modifier.addOutput<connection_types::TransformMessage>("Filtered Transformation");
    output_text_ = node_modifier.addOutput<std::string>("String");  // create a debug output
}

void TransformFilter::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("filter size", 1, 10000, 200, 100));
}

void TransformFilter::process()
{
    filter_size_ = (unsigned int)readParameter<int>("filter size");

    // Output Varibales
    double x, y, z;
    double roll, pitch, yaw;
    std::stringstream stringstream;

    // Get data from input
    TransformMessage::ConstPtr trafo_msg = msg::getMessage<TransformMessage>(input_transform_);
    tf::Transform tf_raw = trafo_msg->value;
    tf::Transform tf_filtered;

    // Run the filter Algorithmn
    runFilter(tf_raw, tf_filtered);

    // Convert the Result to x,y,z r,p,y for debug display
    tfToXYZrpy(tf_filtered, x, y, z, roll, pitch, yaw);

    // Publish Output - Debug
    stringstream << "x = " << x << ",y = " << y << ",z = " << z << ",r = " << RAD_TO_DEG(roll) << ",p = " << RAD_TO_DEG(pitch) << ",y = " << RAD_TO_DEG(yaw);
    msg::publish(output_text_, stringstream.str());

    // Publish Output - Transformation
    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = tf_filtered;
    msg::publish(output_transform_, msg);
}

void TransformFilter::tfToXYZrpy(tf::Transform& in, double& x, double& y, double& z, double& roll, double& pitch, double& yaw)
{
    // Extract the origin
    tf::Vector3 origin = in.getOrigin();
    x = origin.getX();
    y = origin.getY();
    z = origin.getZ();

    // Extract the rotation
    tf::Matrix3x3 rotation(in.getRotation());
    rotation.getRPY(roll, pitch, yaw);
}

void TransformFilter::runFilter(tf::Transform& in_new, tf::Transform& out)
{
    double x, y, z;
    double roll, pitch, yaw;

    tfToXYZrpy(in_new, x, y, z, roll, pitch, yaw);

    std::vector<double> out_vector;

    // filter zeros from invalid transformation
    double test = x + y + z + roll + pitch + yaw;
    if (test != 0) {
        // Eigen::Matrix<float, 1, 6> in_vector(x, y, z, roll, pitch, yaw); // row
        // vector
        std::vector<double> in_vector;
        in_vector.push_back(x);
        in_vector.push_back(y);
        in_vector.push_back(z);

        in_vector.push_back(roll);
        in_vector.push_back(pitch);
        in_vector.push_back(yaw);

        // Save new values in median_matrix_
        for (unsigned int i = 0; i < in_vector.size(); i++) {
            // Some thing like a moving average filter
            if (median_matrix_[i].size() < filter_size_) {
                median_matrix_[i].push_back(in_vector[i]);  // append values, if filter is not full
            } else {
                median_matrix_[i][filter_index_] = in_vector[i];  // insert values if filter is full
            }

            // Remove the elements of the back, if the filter size was reduced
            while (filter_size_ < median_matrix_[i].size()) {
                median_matrix_[i].pop_back();
            }
        }
        filter_index_ = (filter_index_ + 1) % filter_size_;  // rotate the index

        // Calculate the mean over the vector
        for (unsigned int i = 0; i < in_vector.size(); i++) {
            out_vector.push_back(mean(median_matrix_[i]));
        }
        out_vector_latch_ = out_vector;
    }

    // Convert x y z , r p y back to Origin an Quaternion
    tf::Vector3 origin(out_vector_latch_.at(0), out_vector_latch_.at(1), out_vector_latch_.at(2));
    out.setOrigin(origin);

    tf::Quaternion rotation = tf::createQuaternionFromRPY(out_vector_latch_.at(3), out_vector_latch_.at(4), out_vector_latch_.at(5));
    out.setRotation(rotation);
}

double TransformFilter::mean(std::vector<double> in)
{
    double sum_of_elems = 0;
    for (std::vector<double>::iterator j = in.begin(); j != in.end(); ++j) {
        sum_of_elems += *j;
    }
    // ainfo << "SIZE " << in.size() << std::endl;
    if (in.size() != 0) {
        return sum_of_elems / in.size();
    } else {
        return 0;
    }
}
