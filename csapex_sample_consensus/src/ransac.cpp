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
                                std::bind(&Ransac::setupRandomGenerator, this));
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

        typename csapex_sample_consensus::Ransac<PointT>::Ptr sac;
        if(msg::hasMessage(in_indices_)) {
            PointIndecesMessage::ConstPtr in_indices = msg::getMessage<PointIndecesMessage>(in_indices_);
            sac.reset(new csapex_sample_consensus::Ransac<PointT>(in_indices->value->indices, params, rng_));
        } else {
            std::vector<int> indices;
            getIndices<PointT>(cloud, indices);
            sac.reset(new csapex_sample_consensus::Ransac<PointT>(indices, params, rng_));
        }

        pcl::PointIndices outliers;
        pcl::PointIndices inliers;
        inliers.header = cloud->header;
        outliers.header = cloud->header;
        if(fit_multiple_models_) {
            outliers.indices = sac->getIndices();
            int model_searches = 0;
            while(outliers.indices.size() >= minimum_residual_cloud_size_) {
                auto working_model = model->clone();
                sac->computeModel(working_model);
                if(working_model) {
                    inliers.indices.clear();
                    outliers.indices.clear();
                    working_model->getInliersAndOutliers(model_search_distance_, inliers.indices, outliers.indices);

                    if(inliers.indices.size() > minimum_model_cloud_size_)
                        out_inliers->emplace_back(inliers);
                    else
                        out_outliers->emplace_back(inliers);

                    sac->setIndices(outliers.indices);
                }
                ++model_searches;
                if(maximum_model_count_ != -1 && model_searches >= maximum_model_count_)
                    break;
            }
            out_outliers->emplace_back(outliers);
        } else {
            sac->computeModel(model);
            if(model) {
                model->getInliersAndOutliers(model_search_distance_, inliers.indices, outliers.indices);

                if(inliers.indices.size() > minimum_model_cloud_size_) {
                    out_inliers->emplace_back(inliers);
                } else {
                    out_outliers->emplace_back(inliers);
                }
                out_outliers->emplace_back(outliers);
            }
        }

        msg::publish<GenericVectorMessage, pcl::PointIndices>(out_inlier_indices_, out_inliers);
        msg::publish<GenericVectorMessage, pcl::PointIndices>(out_outlier_indices_, out_outliers);
        msg::publish<GenericVectorMessage, ModelMessage>(out_models_, out_models);
    }

protected:
    double inlier_start_probability_;
    int    maximum_sampling_retries_;

    std::default_random_engine rng_;    /// keep the random engine alive for better number generation

    inline void setupRandomGenerator()
    {
        int seed = readParameter<int>("random seed");
        if(seed >= 0) {
            rng_ = std::default_random_engine(seed);
        } else {
            std::random_device rd;
            rng_ = std::default_random_engine(rd());
        }
    }

    inline void fillParamterObject(csapex_sample_consensus::RansacParameters &params)
    {
        SampleConsensus::fillParamterObject(params);
        params.inlier_start_probability = inlier_start_probability_;
        params.maximum_sampling_retries = maximum_sampling_retries_;
    }
};
}

CSAPEX_REGISTER_CLASS(csapex::Ransac, csapex::Node)
