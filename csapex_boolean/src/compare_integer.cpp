
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class CompareInteger : public Node
{
public:
    enum class Operation
    {
        GT,
        GTE,
        EQ,
        NEQ,
        LTE,
        LT
    };

    CompareInteger()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_a = modifier.addInput<int>("A");
        in_b = modifier.addInput<int>("B");

        out = modifier.addOutput<bool>("A OP B");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        std::map<std::string, Operation> operations
        {
            {"A > B", Operation::GT},
            {"A >= B", Operation::GTE},
            {"A == B", Operation::EQ},
            {"A != B", Operation::NEQ},
            {"A <= B", Operation::LTE},
            {"A < B", Operation::LT},
        };
        params.addParameter(param::factory::declareParameterSet("operation",
                                                                         operations,
                                                                         Operation::EQ),
                            operation_);
    }

    void process() override
    {
        int a = msg::getValue<int>(in_a);
        int b = msg::getValue<int>(in_b);

        bool res;

        switch (operation_) {
        case Operation::GT:
            res = a > b;
            break;
        case Operation::GTE:
            res = a >= b;
            break;
        case Operation::EQ:
            res = a == b;
            break;
        case Operation::NEQ:
            res = a != b;
            break;
        case Operation::LTE:
            res = a <= b;
            break;
        case Operation::LT:
            res = a < b;
            break;
        }

        msg::publish(out, res);
    }

private:
    Input* in_a;
    Input* in_b;
    Output* out;

    Operation operation_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::CompareInteger, csapex::Node)

