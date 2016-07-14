#ifndef ELCH_H
#define ELCH_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace csapex {
class ELCH : public Node
{
public:
    ELCH();

    struct PCLELCH {
        typedef std::shared_ptr<PCLELCH> Ptr;
    };

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input  *in_cloud_;
    Output *out_cloud_;

    PCLELCH::Ptr elch_;


};
}

#endif // ELCH_H
