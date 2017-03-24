/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex_ml/features_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/register_apex_plugin.h>

#include <pcl/point_types.h>

namespace csapex {
class FeaturesMessageToLabelledPointCloud : public csapex::Node
{
public:
    using FeaturesMessage = csapex::connection_types::FeaturesMessage;
    using PointCloudMessage = csapex::connection_types::PointCloudMessage;
    using GenericVectorMessage = csapex::connection_types::GenericVectorMessage;


    FeaturesMessageToLabelledPointCloud() = default;

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        in_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("FeaturesMessage");
        out_ = node_modifier.addOutput<PointCloudMessage>("Labelled PointCloud");
    }
    virtual void setupParameters(Parameterizable &parameters) override
    {
    }

    virtual void process() override
    {
        std::shared_ptr<std::vector<FeaturesMessage> const> in =
                msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_);
        PointCloudMessage::Ptr out(new PointCloudMessage("feature", in->front().stamp_micro_seconds));
        pcl::PointCloud<pcl::PointXYZL>::Ptr points(new pcl::PointCloud<pcl::PointXYZL>);

        for(const auto &f : *in) {
            pcl::PointXYZL p;
            switch(f.value.size()) {
            case 3:
                p.z = f.value[2];
            case 2:
                p.y = f.value[1];
            case 1:
                p.z = f.value[0];
                break;
            default:
                throw std::runtime_error("Currently only low dimenisonal features are supported!");
            }
            p.label = f.classification;
            points->push_back(p);
        }

        out->value = points;
        msg::publish(out_, out);
    }

private:
    Input  *in_;
    Output *out_;

};
}

CSAPEX_REGISTER_CLASS(csapex::FeaturesMessageToLabelledPointCloud, csapex::Node)
