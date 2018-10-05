#ifndef TRANSFORM_FROM_MODELS_H
#define TRANSFORM_FROM_MODELS_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/model_message.h>

/// SYSTEM
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace csapex
{
class TransformFromModels : public csapex::Node
{
public:
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> StlVectorEigenVector3d;

public:
    TransformFromModels();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

private:
    Input* input_models_ref_;
    Input* input_models_new_;
    Output* output_;
    Output* output_text_;  // debug output for text

    double param_apex_height_;
    double param_cone_angle_;

    /** Get interresting points from models
     * @brief This function takes a vector of models and returns a vector of
     * coordinate of the needed points for further calcualtion For a cone it
     * returns the apex corrdinates for the circle it calculates also the apex
     * coordinates of the cone
     * @param[in] models a vector with all the models of the segmentation
     * @return a vector with the cordinates of the interresting points
     */
    StlVectorEigenVector3d getInterestingPointsFromModels(std::shared_ptr<std::vector<ModelMessage> const> models);

    /** Match sides of triangle
     * @brief compares the sides of a triangle defined by 3 points
     * @param points1 three points of the first triangle
     * @param points2 three points of the second triangle
     *
     * @return the shift offset that makes the points match
     */
    int matchSidesOfTriangles(const StlVectorEigenVector3d& points1, const StlVectorEigenVector3d& points2);

    /** Calculate the euklidian distance of two points in 3d Space
     * @return euklidian distance in meter
     */
    double euklidianDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

    /** calculate euler angles from a Rotationmatrix R
     * @brief calculates the euler angles from a rotatin matrix R based on the
     * example from Slabaugh
     * http://www.soi.city.ac.uk/~sbbh653/publications/euler.pdf
     * @param[in] R  3 x 3 rotation matrix
     * @param[out] psi out roation angle around x-axis
     * @param[out] theta out roation angle around y-axis
     * @param[out] phi roation angle around z-axis
     */
    void eulerAnglesFromRotationMatrix(const Eigen::Matrix3d& R, double& psi, double& theta, double& phi);

    // Find Transformation from matched triangels
    Eigen::Matrix4d calculateTransformation(const StlVectorEigenVector3d& points_ref, const StlVectorEigenVector3d& points_new, int offset);
    Eigen::Matrix4d threePointsToTransformation(const StlVectorEigenVector3d& points);
    // Optimize transform over time see: http://www.cs.cmu.edu/~ranjith/lcct.html
};
}  // namespace csapex
#endif  // TRANSFORM_FROM_MODELS_H
