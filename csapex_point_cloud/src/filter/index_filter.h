#ifndef INDEX_FILTER_H
#define INDEX_FILTER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class IndexFilter : public Node
{
public:
    IndexFilter();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

protected:
    Input*  input_cloud_;
    Input*  indeces_input_;
    Output* output_cloud_;

};
}
#endif // INDEX_FILTER_H
