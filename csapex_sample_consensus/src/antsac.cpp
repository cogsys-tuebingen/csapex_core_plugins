/// PROJECT
#include "sample_consensus.hpp"

namespace csapex {
using namespace connection_types;

class Antsac : public SampleConsensus
{
public:
    Antsac() = default;

    virtual void setupParameters(Parameterizable &parameters) override
    {
        SampleConsensus::setupParameters(parameters);

        parameters.addParameter(param::ParameterFactory::declareRange("inlier start probability", 0.01, 1.0, 0.9, 0.01),
                                inlier_start_probability_);
        parameters.addParameter(param::ParameterFactory::declareValue("random seed", -1),
                                random_seed_);
        parameters.addParameter(param::ParameterFactory::declareRange("maximum sampling retries", 1, 1000, 100, 1),
                                maximum_sampling_retries_);

        parameters.addParameter(param::ParameterFactory::declareRange("rho", 0.0, 1.0, 0.9, 0.01),
                                rho_);
        parameters.addParameter(param::ParameterFactory::declareRange("alpha", 0.01, 10.0, 0.1, 0.01),
                                alpha_);
        parameters.addParameter(param::ParameterFactory::declareRange("theta", 0.01, 10.0, 0.025, 0.001),
                                theta_);
    }

    virtual void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
        boost::apply_visitor (PointCloudMessage::Dispatch<Antsac>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        std::shared_ptr<std::vector<pcl::PointIndices> > out_inliers(new std::vector<pcl::PointIndices>);
        std::shared_ptr<std::vector<pcl::PointIndices> > out_outliers(new std::vector<pcl::PointIndices>);
        std::shared_ptr<std::vector<ModelMessage> >      out_models(new std::vector<ModelMessage>);

//        auto model = getModel<PointT>(cloud);
//        csapex_sample_consensus::AntsacParameters params;
//        fillParamterObject(params);

//        auto sac = csapex_sample_consensus::Antsac<PointT>(cloud->size(), params);
//        sac.computeModel(model);

//        if(model) {
//            pcl::PointIndices outliers;
//            pcl::PointIndices inliers;
//            inliers.header = cloud->header;
//            outliers.header = cloud->header;
//            model->getInliersAndOutliers(0.1f, inliers.indices, outliers.indices);
//            out_inliers->emplace_back(inliers);
//            out_outliers->emplace_back(outliers);
//        }

        msg::publish<GenericVectorMessage, pcl::PointIndices>(out_inlier_indices_, out_inliers);
        msg::publish<GenericVectorMessage, pcl::PointIndices>(out_outlier_indices_, out_outliers);
        msg::publish<GenericVectorMessage, ModelMessage>(out_models_, out_models);
    }
protected:
    double inlier_start_probability_;
    int    random_seed_;
    int    maximum_sampling_retries_;

    double rho_;
    double alpha_;
    double theta_;

};
}

CSAPEX_REGISTER_CLASS(csapex::Antsac, csapex::Node)

