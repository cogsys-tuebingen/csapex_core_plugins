
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// COMPONENT
#include <csapex_math/msg/linear_matrix_message.h>
#include <csapex_math/param/factory.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class MatrixInput : public Node
{
public:
    MatrixInput()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        out_ = modifier.addOutput<LinearMatrixMessage>("matrix");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        math::linear::Matrix default_matrix{ 2, 2, { 1.0, 0.0, 0.0, 1.0 } };

        params.addParameter(param::factory::declareRange<int>("rows", 1, 8, default_matrix.rows(), 1), [this](param::Parameter* p) {
            auto matrix = readParameter<math::linear::Matrix>("matrix");
            matrix.resize(p->as<int>(), matrix.cols());
            setParameter("matrix", math::linear::Matrix(matrix));
        });
        params.addParameter(param::factory::declareRange<int>("cols", 1, 8, default_matrix.cols(), 1), [this](param::Parameter* p) {
            auto matrix = readParameter<math::linear::Matrix>("matrix");
            matrix.resize(matrix.rows(), p->as<int>());
            setParameter("matrix", math::linear::Matrix(matrix));
        });
        params.addParameter(param::factory::declareMatrix("matrix", default_matrix));
    }

    void process() override
    {
        msg::publish(out_, std::make_shared<LinearMatrixMessage>(readParameter<math::linear::Matrix>("matrix")));
    }

private:
    Output* out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::MatrixInput, csapex::Node)
