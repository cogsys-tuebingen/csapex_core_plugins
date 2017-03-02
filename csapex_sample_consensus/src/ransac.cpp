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

        typename csapex_sample_consensus::SampleConsensusModel<PointT>::Ptr model(new csapex_sample_consensus::ModelPlane<PointT>(cloud));
        typename csapex_sample_consensus::Ransac<PointT>::Ptr sac(new csapex_sample_consensus::Ransac<PointT>(cloud->size(),
                                                                                                              csapex_sample_consensus::RansacParameters()));

        sac->computeModel(model);

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
};
}

CSAPEX_REGISTER_CLASS(csapex::Ransac, csapex::Node)
