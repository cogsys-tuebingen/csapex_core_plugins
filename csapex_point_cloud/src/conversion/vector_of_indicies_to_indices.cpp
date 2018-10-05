
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

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class VectorOfIndiciesToIndices : public Node
{
public:
    VectorOfIndiciesToIndices()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage, pcl::PointIndices>("PointIndices");
        out_ = modifier.addOutput<PointIndicesMessage>("PointIndices");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        auto vector = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_);

        PointIndicesMessage::Ptr out(new PointIndicesMessage);
        if (!vector->empty()) {
            out->value->header = vector->front().header;
            for (auto indices : *vector) {
                out->value->indices.insert(out->value->indices.end(), indices.indices.begin(), indices.indices.end());
            }
        }
        msg::publish(out_, out);
    }

private:
    Input* in_;
    Output* out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::VectorOfIndiciesToIndices, csapex::Node)
