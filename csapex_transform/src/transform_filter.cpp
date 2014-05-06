/// HEADER
#include "transform_filter.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING


#define RAD_TO_DEG(x) ((x) * 57.29578)

CSAPEX_REGISTER_CLASS(csapex::TransformFilter, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformFilter::TransformFilter()
{
    addTag(Tag::get("Transform"));
    addTag(Tag::get("Filter"));

    median_matrix_.resize(6);
}

void TransformFilter::setup()
{
    input_transform_ = addInput<connection_types::TransformMessage>("Raw Transformation");

    output_transform_ = addOutput<connection_types::TransformMessage>("Filtered Transformation");
    output_text_ = addOutput<DirectMessage<std::string> >("String"); // create a debug output
}

void TransformFilter::process()
{
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
    DirectMessage<std::string>::Ptr text_msg(new DirectMessage<std::string>);
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
    rotation.getEulerYPR(roll, pitch, yaw);
}

void TransformFilter::runFilter(tf::Transform& in_new, tf::Transform& out)
{
    double x,y,z;
    double roll, pitch, yaw;

    tfToXYZrpy(in_new, x, y, z, roll, pitch, yaw);

    // Check if not all values are zero
    std::vector<double> out_vector;
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
            median_matrix_[i].push_back(in_vector[i]);
        }

        // Calculate the mean over the vector
        for (unsigned int i = 0; i< in_vector.size(); i++) {
            out_vector.push_back(mean(median_matrix_[i]));
        }

    } else {
        // Return 0 for all values if the vector was 0 before
        for (unsigned int i = 0; i< 6; i++) {
            out_vector.push_back(0.0);
        }
    }

    // Convert x y z , r p y back to Origin an Quaternion
    tf::Vector3 origin(out_vector.at(0), out_vector.at(1), out_vector.at(2));
    out.setOrigin(origin);

    tf::Quaternion rotation = tf::createQuaternionFromRPY(out_vector.at(3), out_vector.at(4), out_vector.at(5));
    out.setRotation(rotation);
}

double TransformFilter::mean(std::vector<double> in)
{
    double sum_of_elems = 0;
    for(std::vector<double>::iterator j=in.begin();j!=in.end();++j) {
        sum_of_elems += *j;
    }
    std::cout << "SIZE " << in.size() << std::endl;
    if (in.size() != 0) {
        return sum_of_elems / in.size();
    } else {
        return 0;
    }
}
