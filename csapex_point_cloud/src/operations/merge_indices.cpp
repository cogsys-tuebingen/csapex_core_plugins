/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/indeces_message.h>
#include <csapex/msg/io.h>
#include <pcl/PointIndices.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex {

using namespace connection_types;
class MergeIndices : public Node
{
public:
    MergeIndices()
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_indices_ = node_modifier.addInput<GenericVectorMessage, pcl::PointIndices>("Vector of Indices");
        output_indices_ = node_modifier.addOutput<PointIndecesMessage>("Indices");
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
    }

    virtual void process() override
    {

        std::shared_ptr<std::vector<pcl::PointIndices> const> in_indices_msg = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(input_indices_);
        PointIndecesMessage::Ptr out_indices_msg(new PointIndecesMessage);

        out_indices_msg->value.reset(new pcl::PointIndices);
        std::vector<int> &indices = out_indices_msg->value->indices;
        for(const pcl::PointIndices &p : *in_indices_msg) {
            indices.insert(indices.end(), p.indices.begin(), p.indices.end());
        }


        msg::publish(output_indices_, out_indices_msg);
    }

private:
    Input*  input_indices_;
    Output* output_indices_;


};
CSAPEX_REGISTER_CLASS(csapex::MergeIndices, csapex::Node)
}
