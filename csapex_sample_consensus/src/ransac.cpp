/// PROJECT
#include "sample_consensus.hpp"

namespace csapex {
using namespace connection_types;

class Ransac : public SampleConsensus
{
public:
    Ransac() = default;

    virtual void setupParameters(Parameterizable &parameters) override
    {
        SampleConsensus::setupParameters(parameters);

        parameters.addParameter(param::ParameterFactory::declareRange("inlier start probability", 0.01, 1.0, 0.9, 0.01),
                                inlier_start_probability_);
        parameters.addParameter(param::ParameterFactory::declareValue("random seed", -1),
                                random_seed_);
        parameters.addParameter(param::ParameterFactory::declareRange("maximum sampling retries", 1, 1000, 100, 1),
                                maximum_sampling_retries_);
    }

    virtual void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
        boost::apply_visitor (PointCloudMessage::Dispatch<Ransac>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        std::shared_ptr<std::vector<pcl::PointIndices> > out_inliers(new std::vector<pcl::PointIndices>);
        std::shared_ptr<std::vector<pcl::PointIndices> > out_outliers(new std::vector<pcl::PointIndices>);
        std::shared_ptr<std::vector<ModelMessage> >      out_models(new std::vector<ModelMessage>);

        auto model = getModel<PointT>(cloud);
        csapex_sample_consensus::RansacParameters params;
        fillParamterObject(params);

        auto sac = csapex_sample_consensus::Ransac<PointT>(cloud->size(), params);
        sac.computeModel(model);

        if(model) {
            pcl::PointIndices outliers;
            pcl::PointIndices inliers;
            inliers.header = cloud->header;
            outliers.header = cloud->header;
            model->getInliersAndOutliers(0.1f, inliers.indices, outliers.indices);
            out_inliers->emplace_back(inliers);
            out_outliers->emplace_back(outliers);
        }

        msg::publish<GenericVectorMessage, pcl::PointIndices>(out_inlier_indices_, out_inliers);
        msg::publish<GenericVectorMessage, pcl::PointIndices>(out_outlier_indices_, out_outliers);
        msg::publish<GenericVectorMessage, ModelMessage>(out_models_, out_models);
    }

protected:
    double inlier_start_probability_;
    int    random_seed_;
    int    maximum_sampling_retries_;

    inline void fillParamterObject(csapex_sample_consensus::RansacParameters &params)
    {
        SampleConsensus::fillParamterObject(params);
        params.inlier_start_probability = inlier_start_probability_;
        params.random_seed = random_seed_;
        params.maximum_sampling_retries = maximum_sampling_retries_;
    }
};
}

CSAPEX_REGISTER_CLASS(csapex::Ransac, csapex::Node)
