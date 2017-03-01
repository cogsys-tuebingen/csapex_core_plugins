#ifndef SACFIT2_H
#define SACFIT2_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/model_message.h>



namespace csapex {
class SampleConsensus : public csapex::Node
{
public:
    SampleConsensus();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input*  in_cloud_;
    Input*  in_indices_;
    Output* out_models_;
    Output* out_inlier_indices_;
    Output* out_outlier_indices_;

};
}


#endif // SACFIT2_H