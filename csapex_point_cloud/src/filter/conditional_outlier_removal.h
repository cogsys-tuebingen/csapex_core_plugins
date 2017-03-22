#ifndef STATISTICAL_OUTLIER_REMOVAL_H
#define STATISTICAL_OUTLIER_REMOVAL_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex {
class ConditionalOutlierRemoval : public Node
{
public:
    ConditionalOutlierRemoval();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    enum ConditionType {AND, OR};

    Input*  input_;
    Output* output_;

    void update();

    ConditionType   type_;
    int             conditions_;
    Eigen::Vector2d x_range_;
    Eigen::Vector2d y_range_;
    Eigen::Vector2d z_range_;
    bool          keep_organized_;


};
}
#endif // STATISTICAL_OUTLIER_REMOVAL_H
