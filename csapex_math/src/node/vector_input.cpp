
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
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
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        std::vector<double> default_vector {0.0, 0.0, 0.0};

        params.addParameter(param::factory::declareRange<int>("dimension", 1, 8, default_vector.size(), 1), [this](param::Parameter* p){
            vector_.resize(p->as<int>());
            setParameter("vector", math::linear::Vector(vector_));
        });
        params.addParameter(param::factory::declareVector("vector", default_vector), vector_);
    }

    void process() override
    {
    }

private:
    Output* out_;

    std::vector<double> vector_;
};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::VectorInput, csapex::Node)

