
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class VectorSize : public Node
{
public:
    VectorSize()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage>("Input");
        out_ = modifier.addOutput<int>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareOutputText("count"));
    }

    void process() override
    {
        GenericVectorMessage::ConstPtr vec = msg::getMessage<GenericVectorMessage>(in_);

        int size = vec->nestedValueCount();

        setParameter("count", std::to_string(size));

        msg::publish(out_, size);
    }

private:
    Input* in_;
    Output* out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::VectorSize, csapex::Node)
