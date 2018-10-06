
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// COMPONENT
#include <csapex_math/msg/linear_vector_message.h>
#include <csapex_math/param/factory.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class VectorInput : public Node
{
public:
    VectorInput()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        out_ = modifier.addOutput<LinearVectorMessage>("vector");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        std::vector<double> default_vector{ 0.0, 0.0, 0.0 };

        params.addParameter(param::factory::declareRange<int>("dimension", 1, 8, default_vector.size(), 1), [this](param::Parameter* p) {
            auto vector = readParameter<math::linear::Vector>("vector");
            vector.resize(p->as<int>(), 1);
            setParameter("vector", math::linear::Vector(vector));
        });
        params.addParameter(param::factory::declareVector("vector", default_vector));
    }

    void process() override
    {
        msg::publish(out_, std::make_shared<LinearVectorMessage>(readParameter<math::linear::Vector>("vector")));
    }

private:
    Output* out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::VectorInput, csapex::Node)
