#ifndef SAMPLE_CONSENSUS_HPP
#define SAMPLE_CONSENSUS_HPP

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>

#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_value_message.hpp>

#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/model_message.h>
#include <csapex_point_cloud/msg/indeces_message.h>
#include <csapex_point_cloud/msg/normals_message.h>

#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

#include <pcl/point_representation.h>

#include <boost/mpl/for_each.hpp>

#include <Eigen/Core>

#include <csapex_sample_consensus/csapex_sample_consensus.hpp>

namespace csapex {
using namespace connection_types;

class SampleConsensus : public csapex::Node
{
public:
    SampleConsensus() = default;

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::ParameterFactory::declareRange("model search distance", 0.0, 10.0, 0.1, 0.001),
                                sac_parameters_.model_search_distance);
        parameters.addParameter(param::ParameterFactory::declareBool("terminate one mean model distance", false),
                                sac_parameters_.use_mean_model_distance);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("maximum mean model distance", 0.0, 10.0, 0.05, 0.01),
                                           [this](){return sac_parameters_.use_mean_model_distance;},
        sac_parameters_.mean_model_distance);
        //        parameters.addParameter(param::ParameterFactory::declareBool("optimize model coefficients", false),
        //                                ransac_parameters_.optimize_model_coefficients);
        parameters.addParameter(param::ParameterFactory::declareRange("point skip", 0, 10, 0, 1),
                                point_skip_);
        parameters.addParameter(param::ParameterFactory::declareRange("maximum iterations", 1, 100000, 100, 1),
                                sac_parameters_.maximum_iterations);

        parameters.addParameter(param::ParameterFactory::declareRange("minimum model cloud size", 1, 100000, 1000, 1),
                                minimum_model_cloud_size_);

        parameters.addParameter(param::ParameterFactory::declareBool("find multiple models", false),
                                fit_multiple_models_);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("minimum residual cloud size", 0, 1000, 100, 1),
                                           [this](){return fit_multiple_models_;},
        minimum_residual_cloud_size_);
        parameters.addConditionalParameter(param::ParameterFactory::declareRange("maximum model count", -1, 100, 5, 1),
                                           [this](){return fit_multiple_models_;},
        maximum_model_count_);

        std::map<std::string, int> model_types =
        {{"PLANE", PLANE},
         {"NORMAL_PLANE", NORMAL_PLANE},
         {"PARALLEL_NORMAL_PLANE", PARALLEL_NORMAL_PLANE}};

        parameters.addParameter(param::ParameterFactory::declareParameterSet("model type", model_types, (int) PLANE),
                                model_type_);

        /// actually, this can be done better ! - temporary
        auto uses_axis = [this] () {
            return model_type_ == PARALLEL_NORMAL_PLANE;
        };

        auto uses_normals = [this]() {
            return model_type_ == PARALLEL_NORMAL_PLANE || NORMAL_PLANE;
        };


        parameters.addConditionalParameter(param::ParameterFactory::declareValue("axis/x", 0.0),
                                           uses_axis,
                                           axis_(0));
        parameters.addConditionalParameter(param::ParameterFactory::declareValue("axis/y", 0.0),
                                           uses_axis,
                                           axis_(1));
        parameters.addConditionalParameter(param::ParameterFactory::declareValue("axis/z", 1.0),
                                           uses_axis,
                                           axis_(2));

        parameters.addConditionalParameter(param::ParameterFactory::declareValue("normal/distance_weight", 0.0),
                                           uses_normals,
                                           normal_distance_weight_);

        parameters.addConditionalParameter(param::ParameterFactory::declareAngle("normal/angle_distance",
                                                                                 0.0),
                                           uses_axis,
                                           angle_eps_);
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_cloud_           = node_modifier.addInput<PointCloudMessage>("PointCloud");
        in_indices_         = node_modifier.addOptionalInput<PointIndecesMessage>("Indices"); // optional input
        in_normals_         = node_modifier.addOptionalInput<NormalsMessage>("Normals");      // optional input

        out_models_         = node_modifier.addOutput<GenericVectorMessage, ModelMessage >("Models");
        out_inlier_indices_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Model Points");
        out_outlier_indices_= node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("Rejected Points");
    }

protected:
    enum ModelType {PLANE, NORMAL_PLANE, PARALLEL_NORMAL_PLANE};

    int                  model_type_;

    csapex_sample_consensus::Parameters sac_parameters_;

    int                  point_skip_;
    int                  termination_criteria_;
    int                  minimum_fit_size_;
    int                  minimum_model_cloud_size_;

    bool                 fit_multiple_models_;
    int                  minimum_residual_cloud_size_;
    int                  maximum_model_count_;

    double               normal_distance_weight_;
    Eigen::Vector3d      axis_;
    double               angle_eps_;


    Input*               in_cloud_;
    Input*               in_indices_;
    Input*               in_normals_;
    Output*              out_models_;
    Output*              out_inlier_indices_;
    Output*              out_outlier_indices_;

    template<typename PointT>
    typename csapex_sample_consensus::models::Model<PointT>::Ptr getModel(typename pcl::PointCloud<PointT>::ConstPtr &cloud)
    {
        typename csapex_sample_consensus::models::Model<PointT>::Ptr model;

        switch((ModelType) model_type_) {
        case PLANE:
            model.reset(new csapex_sample_consensus::models::Plane<PointT>(cloud));
            break;
        case NORMAL_PLANE:
        {
            pcl::PointCloud<pcl::Normal>::ConstPtr normals;
            getNormals(normals);
            if(!normals)
                throw std::runtime_error("The selected setting requires normals!");

            model.reset(new csapex_sample_consensus::models::NormalPlane<PointT, pcl::Normal>(cloud,
                                                                                              normals,
                                                                                              normal_distance_weight_));

        }
            break;
        case PARALLEL_NORMAL_PLANE:
        {
            pcl::PointCloud<pcl::Normal>::ConstPtr normals;
            getNormals(normals);
            if(!normals)
                throw std::runtime_error("The selected setting requires normals!");

            auto *pnp_model = new csapex_sample_consensus::models::ParallelNormalPlane<PointT, pcl::Normal>(cloud,
                                                                                                            normals,
                                                                                                            normal_distance_weight_);
            pnp_model->setAxis(pcl::Normal(axis_(0), axis_(1), axis_(2)),
                               angle_eps_);
            model.reset(pnp_model);
        }
            break;
        default:
            throw std::runtime_error("[SampleConsensus]: Unknown model type!");
        }

        return model;
    }

    void getNormals(pcl::PointCloud<pcl::Normal>::ConstPtr &normals)
    {
        if(msg::hasMessage(in_normals_)) {
            NormalsMessage::ConstPtr in_normals = msg::getMessage<NormalsMessage>(in_normals_);
            normals = in_normals->value;
        }
    }

    template<typename PointT>
    void getIndices(typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                    std::vector<int> &indices)
    {
        const static pcl::DefaultPointRepresentation<PointT> pr;
        const std::size_t size = cloud->size();
        indices.reserve(size);

        const std::size_t step = 1 + point_skip_;
        for(std::size_t i = 0 ; i < size; i+=step) {
            if(pr.isValid(cloud->at(i))) {
                indices.emplace_back(i);
            }
        }
    }

    void getInidicesFromInput(std::vector<int> &indices)
    {
        if(msg::hasMessage(in_indices_)) {
            PointIndecesMessage::ConstPtr in_indices_msg = msg::getMessage<PointIndecesMessage>(in_indices_);
            const std::vector<int> &in_indices = in_indices_msg->value->indices;
            if(point_skip_ > 0) {
                const std::size_t step = 1 + point_skip_;
                const std::size_t size = in_indices.size();
                indices.reserve(size);
                for(std::size_t i = 0 ; i < size ; i+= step) {
                    indices.emplace_back(in_indices[i]);
                }
            } else {
                indices = in_indices;
            }
        }
    }
};
}

#endif // SAMPLE_CONSENSUS_HPP
