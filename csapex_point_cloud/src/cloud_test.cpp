/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/msg/io.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/event.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex {

using namespace connection_types;


class CloudTest : public Node
{
public:
    CloudTest()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");

        trigger_empty_ = node_modifier.addEvent("empty");
        trigger_not_empty_ = node_modifier.addEvent("not empty");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        addParameter(param::ParameterFactory::declareRange("min count for not empty", 0, 1024, 1, 1), min_count_);
    }

    virtual void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));
        boost::apply_visitor (PointCloudMessage::Dispatch<CloudTest>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        int count = 0;
        for(const PointT& pt : cloud->points) {
            if(pcl::isFinite(pt)) {
                ++count;
            }

            if(count >= min_count_) {
                trigger_not_empty_->trigger();
                return;
            }
        }

        trigger_empty_->trigger();
    }

private:
    Input*  input_cloud_;
    Event* trigger_empty_;
    Event* trigger_not_empty_;

    int min_count_;
};
CSAPEX_REGISTER_CLASS(csapex::CloudTest, csapex::Node)
}
