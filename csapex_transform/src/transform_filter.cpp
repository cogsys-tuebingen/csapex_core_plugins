/// HEADER
#include "transform_filter.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>


#define RAD_TO_DEG(x) ((x) )
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

void TransformFilter::setup()
{
    input_transform_ = modifier_->addInput<connection_types::TransformMessage>("Raw Transformation");

    output_transform_ = modifier_->addOutput<connection_types::TransformMessage>("Filtered Transformation");
    output_text_ = modifier_->addOutput<GenericValueMessage<std::string> >("String"); // create a debug output

    addParameter(param::ParameterFactory::declare("filter size", 1, 10000, 200, 100));
}

void TransformFilter::process()
{
    filter_size_ =  (unsigned int)readParameter<int>("filter size");

    // Output Varibales
    double x,y,z;
    double roll, pitch, yaw;
    std::stringstream stringstream;

    // Get data from input
    TransformMessage::Ptr trafo_msg = input_transform_->getMessage<TransformMessage>();
    tf::Transform tf_raw = trafo_msg->value;
    tf::Transform tf_filtered;


    // Run the filter Algorithmn
    runFilter(tf_raw, tf_filtered);


    // Convert the Result to x,y,z r,p,y for debug display
    tfToXYZrpy(tf_filtered, x, y, z, roll, pitch, yaw);

    // Publish Output - Debug
    stringstream << "x = " << x << ",y = " << y << ",z = " << z
                 << ",r = " << RAD_TO_DEG(roll) << ",p = " << RAD_TO_DEG(pitch)<< ",y = " << RAD_TO_DEG(yaw);
    GenericValueMessage<std::string>::Ptr text_msg(new GenericValueMessage<std::string>);
    text_msg->value = stringstream.str();
    output_text_->publish(text_msg);

    // Publish Output - Transformation
    connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
    msg->value = tf_filtered;
    output_transform_->publish(msg);
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
    double x,y,z;
    double roll, pitch, yaw;

    tfToXYZrpy(in_new, x, y, z, roll, pitch, yaw);

    std::vector<double> out_vector;

        // filter zeros from invalid transformation
        double test = x+y+z + roll + pitch + yaw;
        if (test != 0) {

            //Eigen::Matrix<float, 1, 6> in_vector(x, y, z, roll, pitch, yaw); // row vector
            std::vector<double> in_vector;
            in_vector.push_back(x);
            in_vector.push_back(y);
            in_vector.push_back(z);

            in_vector.push_back(roll);
            in_vector.push_back(pitch);
            in_vector.push_back(yaw);

            // Save new values in median_matrix_
            for (unsigned int i = 0; i< in_vector.size(); i++) {
                // Some thing like a moving average filter
                if (median_matrix_[i].size() < filter_size_) {
                    median_matrix_[i].push_back(in_vector[i]); // append values, if filter is not full
                } else {
                    median_matrix_[i][filter_index_] = in_vector[i]; // insert values if filter is full
                }

                // Remove the elements of the back, if the filter size was reduced
                while(filter_size_ < median_matrix_[i].size()) {
                    median_matrix_[i].pop_back();
                }
            }
            filter_index_ = (filter_index_ + 1) % filter_size_; // rotate the index

            // Calculate the mean over the vector
            for (unsigned int i = 0; i< in_vector.size(); i++) {
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
    for(std::vector<double>::iterator j=in.begin();j!=in.end();++j) {
        sum_of_elems += *j;
    }
   // ainfo << "SIZE " << in.size() << std::endl;
    if (in.size() != 0) {
        return sum_of_elems / in.size();
    } else {
        return 0;
    }
}
