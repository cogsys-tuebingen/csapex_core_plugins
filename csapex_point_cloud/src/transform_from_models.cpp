#include "transform_from_models.h"

/// COMPONENT
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex_core_plugins/vector_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <tf/tf.h>

//#define RAD_TO_DEG(x) ((x) * 57.29578)
#define RAD_TO_DEG(x) ((x))

CSAPEX_REGISTER_CLASS(csapex::TransformFromModels, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

TransformFromModels::TransformFromModels()
{
}

void TransformFromModels::setup()
{
    input_models_ref_ = modifier_->addInput<GenericVectorMessage, ModelMessage >("Reference Models");
    input_models_new_ = modifier_->addInput<GenericVectorMessage, ModelMessage >("New Models");
    output_ = modifier_->addOutput<connection_types::TransformMessage>("Transformation");
    output_text_ = modifier_->addOutput<std::string>("String"); // create a debug output


    addParameter(param::ParameterFactory::declareRange("Apex height", 0.0, 2.0, 0.5, 0.01));
    addParameter(param::ParameterFactory::declareRange("Cone angle", 0.0001, 3.2, 0.5, 0.001));
}

void TransformFromModels::process()
{
    param_apex_height_ = readParameter<double>("Apex height");
    param_cone_angle_ = readParameter<double>("Cone angle");

    // Read Inputs
    boost::shared_ptr<std::vector<ModelMessage> const> models_ref = input_models_ref_->getMessage<GenericVectorMessage, ModelMessage>();
    boost::shared_ptr<std::vector<ModelMessage> const> models_new = input_models_new_->getMessage<GenericVectorMessage, ModelMessage>();


    // Process data
    std::vector<Eigen::Vector3d> points_ref = getInterestingPointsFromModels(models_ref);
    std::vector<Eigen::Vector3d> points_new = getInterestingPointsFromModels(models_new);

    // Output Varibales
    double x,y,z;
    double roll, pitch, yaw;
    std::stringstream stringstream;

    // Check that there are 3 points for each model, so that we can match the trinangles
    if ((points_ref.size() == 3) && (points_new.size() == 3)) {

        int offset = matchSidesOfTriangles(points_ref, points_new);

        Eigen::Matrix4d r_T_n(4,4);
        r_T_n = calculateTransformation(points_ref, points_new, offset);

        // Extract the Translation
        x = r_T_n(0,3);
        y = r_T_n(1,3);
        z = r_T_n(2,3);



        //   // Test the functtion     eulerAnglesFromRotationMatrix()
        //    Eigen::Matrix3d r_test; // test matrix to test the algorimthm
        //    r_test <<       0.5,    -0.1464,  0.8536,
        //                    0.5,     0.8536, -0.1464,
        //                    -0.707,   0.5,     0.5;
        //  eulerAnglesFromRotationMatrix(r_test, roll, pitch, yaw);

        // Extract the rotation
        Eigen::Matrix3d rotation;
        rotation = r_T_n.block<3,3>(0,0); // cut the rotation matrix from the transformation matrix
        eulerAnglesFromRotationMatrix(rotation, roll, pitch, yaw); // Convert Homogenious Coordinates to a Quaternion

        // check for nan and set values to zero
        double test = x + y + z + roll + pitch + yaw; // to test if one is nan
        if (isnan(test)) {
            x = y = z = 0.0;
            roll = pitch = yaw = 0.0;
        }

        // Publish Output - Debug
        stringstream << "x = " << x << ",y = " << y << ",z = " << z
                     << ",r = " << RAD_TO_DEG(roll) << ",p = " << RAD_TO_DEG(pitch)<< ",y = " << RAD_TO_DEG(yaw);
        output_text_->publish(stringstream.str());

        // Publish Output - Transformation
        connection_types::TransformMessage::Ptr msg(new connection_types::TransformMessage);
        msg->value = tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z));
        output_->publish(msg);


    } else {
        ainfo << "points_ref.size() = " << points_ref.size() << " points_new.size() = " << points_new.size() << std::endl;
        stringstream << "points_ref.size() = " << points_ref.size() << " points_new.size() = " << points_new.size() << std::endl;

        // Set output values to zero, if the there are not 3 points for each model
        x = y = z = 0.0;
        roll = pitch = yaw = 0.0;
    }




}

// TODO: This can also be done by tf library
void TransformFromModels::eulerAnglesFromRotationMatrix( Eigen::Matrix3d R, double &psi, double &theta, double &phi)
{
    /* Note: this function is based on the paper http://www.soi.city.ac.uk/~sbbh653/publications/euler.pdf
     * but the second solution is ignored
     * Test the function with
     * R =
     *[[ 0.5    -0.1464  0.8536]
     * [ 0.5     0.8536 -0.1464]
     * [-0.707   0.5     0.5   ]]
     * It shuould return psi = 0.78539816, theta = 0.78524716, phi = 0.78539816
     **/


    if ((R(2,0) != 1) && (R(2,0) != -1))
    {
        theta = -std::asin(R(2,0));
        double c = cos(theta);
        psi = std::atan2(R(2,1)/c, R(2,2)/c);
        phi = std::atan2(R(1,0)/c, R(0,0)/c);
    } else  {
        phi = 0; // infinit solution for phi, just pick zero
        if (R(0,2) == -1){
            theta = M_PI / 2.0;
            psi = phi + std::atan2(R(0,1), R(0,2));
        } else {
            theta = - M_PI / 2.0;
            psi = - phi + std::atan2(R(0,1), R(0,2));
        }
    }

}

std::vector<Eigen::Vector3d> TransformFromModels::getInterestingPointsFromModels(boost::shared_ptr<std::vector<ModelMessage> const> models)
{
    std::vector<Eigen::Vector3d> points;

    // got through all models
    // TODO: there should only be 3 models
    for (std::vector<ModelMessage>::const_iterator it = models->begin(); it != models->end(); it++){
        Eigen::Vector3d interresting_point(0,0,0);
        bool point_ok = false;
        switch(it->model_type) {
            case pcl::SACMODEL_SPHERE:
                interresting_point(0) = it->coefficients->values.at(0);
                interresting_point(1) = it->coefficients->values.at(1);
                interresting_point(2) = it->coefficients->values.at(2);
                point_ok = true;
                break;

            case pcl::SACMODEL_CONE:
                interresting_point(0) = it->coefficients->values.at(0);
                interresting_point(1) = it->coefficients->values.at(1);
                interresting_point(2) = it->coefficients->values.at(2);
                point_ok = true;
                break;

            case pcl::SACMODEL_CIRCLE2D: {
                interresting_point(0) = it->coefficients->values.at(0);
                interresting_point(1) = it->coefficients->values.at(1);
                // Calculate the z coordiantes of the apex, given the radius and the opening angle
                double radius = it->coefficients->values.at(2);
                double tan_result = std::tan(param_cone_angle_/2.0);
                //apex_assert_hard(tan_result != 0);
                double z;
                if (tan_result != 0) {
                    z = radius / tan_result;
                } else {
                    aerr << "Invalid cone angle" << std::endl;
                    z = 0;
                }
                interresting_point(2) = z;
                // Only for Testing
                //interresting_point(2) = 0.5; // set the apex height 50cm above ground
                point_ok = true;
            } break;

            default:
                aerr << "Invalid Model Type: " <<  it->model_type << std::endl;
        } // end switch
        if (point_ok) {
            points.push_back(interresting_point);
        }
    } // end for

    return points;
}

double TransformFromModels::euklidianDistance(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    return std::sqrt((p1(0)-p2(0))*(p1(0)-p2(0)) + (p1(1)-p2(1))*(p1(1)-p2(1)) + (p1(2)-p2(2))*(p1(2)-p2(2)));
}

int TransformFromModels::matchSidesOfTriangles(const std::vector<Eigen::Vector3d> &points1, const std::vector<Eigen::Vector3d> &points2)
{
    // Calculated distances of the points -> that are the sides of the triangles
    std::vector<double> distances1;
    unsigned int point_size = points1.size();
    for (unsigned int i=0; i < point_size; i++) {
        double distance = euklidianDistance(points1.at(i%point_size), points1.at((i+1)%point_size));
        distances1.push_back(distance);
    }

    std::vector<double> distances2;
    point_size = points2.size();
    for (unsigned  int i=0; i < point_size; i++) {
        double distance = euklidianDistance(points2.at(i%point_size), points2.at((i+1)%point_size));
        distances2.push_back(distance);
    }

//    // Test values for debug
//    distances1.push_back(0.71);
//    distances1.push_back(1.54);
//    distances1.push_back(1.17);

//    distances2.push_back(1.17);
//    distances2.push_back(0.71);
//    distances2.push_back(1.64);

    int min_offset = 0;
    if ((distances1.size() == 3) && (distances2.size() == 3)) {
        // find the offset that minimizes the differences of the sides
        double min_sum = 20000.0;
        for (int offset = 0; offset<3; offset++){
            double sum = 0;
            for (int i = 0; i<3; i++) {
                sum += std::pow((distances1.at(i) - distances2.at((i+offset) % 3)),2);
            }
            if (sum < min_sum) {
                min_sum = sum;
                min_offset = offset;
            }
        }
    } else  {
        aerr << "There is an triangle with less than 3 points (error)" << std::endl;
    }
    return min_offset;
}

Eigen::Matrix4d TransformFromModels::calculateTransformation(const std::vector<Eigen::Vector3d> &points_ref, const std::vector<Eigen::Vector3d> &points_new, int offset)
{
    // Shift the reference points according to the offset
    std::vector<Eigen::Vector3d> points_new_shifted;
    for (unsigned int i=0; i < points_new.size(); i++) {
        points_new_shifted.push_back(points_new.at((i+offset)% points_new.size()));
    }

    Eigen::Matrix4d r_T_0;
    r_T_0 = threePointsToTransformation(points_ref);

    Eigen::Matrix4d n_T_0;
    n_T_0 = threePointsToTransformation(points_new_shifted);

    return r_T_0 * n_T_0.inverse(); //orginal
    //return n_T_0 * r_T_0 .inverse(); //that the found transformation is the same as the testone
}

using namespace std;

Eigen::Matrix4d TransformFromModels::threePointsToTransformation(const std::vector<Eigen::Vector3d> &points)
{
    Eigen::Matrix4d transformation;
    transformation.setZero(); // initalize with zeros

    // assert (points.size() == 3);
    if (points.size() > 2) {
        Eigen::Vector3d v1(points.at(0) - points.at(1));
        Eigen::Vector3d v2(points.at(2) - points.at(0));

//      /// THIS IS THE WRONG ASSIGNMENT: first index: row, second: collumn
//        // example for block http://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html
//        transformation.block<1,3>(0,0) = v1;          // x vector of the coordinate system
//        transformation.block<1,3>(1,0) = v2;          // y vector of the coordinate system
//        transformation.block<1,3>(2,0) = v1.cross(v2);          // z vector of the coordinate system
//        transformation.block<1,3>(3,0) = points.at(0); // origin of the coordinate system
//        transformation(3,3) = 1;


        // example for block http://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html
        transformation.block<3,1>(0,0) = v1;          // x vector of the coordinate system
        transformation.block<3,1>(0,1) = v2;          // y vector of the coordinate system
        transformation.block<3,1>(0,2) = v1.cross(v2);          // z vector of the coordinate system
        transformation.block<3,1>(0,3) = points.at(0); // origin of the coordinate system
        transformation(3,3) = 1;
    } else
    {
        ROS_ERROR("TransformFromModels: Not Enough points Found");
    }

    return transformation;
}
