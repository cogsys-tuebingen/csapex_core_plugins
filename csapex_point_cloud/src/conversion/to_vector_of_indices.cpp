/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_point_cloud/msg/indices_message.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace csapex
{
using namespace connection_types;

class ToVectorOfIndices : public Node
{
public:
    void setup(NodeModifier& node_modifier) override
    {
        input_indices_ = node_modifier.addInput<PointIndicesMessage>("PointIndices");
        output_indices_ = node_modifier.addOutput<GenericVectorMessage, pcl::PointIndices>("PointIndices");
    }
    void setupParameters(Parameterizable& parameters) override
    {
    }
    void process() override
    {
        PointIndicesMessage::ConstPtr indices(msg::getMessage<PointIndicesMessage>(input_indices_));
        auto out_indices = std::shared_ptr<std::vector<pcl::PointIndices>>(new std::vector<pcl::PointIndices>);
        out_indices->emplace_back(*(indices->value));
        msg::publish<GenericVectorMessage, pcl::PointIndices>(output_indices_, out_indices);
    }

protected:
    Input* input_indices_;
    Output* output_indices_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::ToVectorOfIndices, csapex::Node)
