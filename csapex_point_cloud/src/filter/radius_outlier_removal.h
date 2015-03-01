#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class RadiusOutlierRemoval : public Node
{
public:
    RadiusOutlierRemoval();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input*  input_cloud_;
    Input*  indeces_input_;
    Output* output_cloud_;
    Output* output_indeces_;

};
}
#endif // STATISTICAL_OUTLIER_REMOVAL_H
