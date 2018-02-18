
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


class BinaryBoolean : public Node
{
public:
    enum class Operation
    {
        AND,
        OR,
        NAND,
        NOR,
        XOR
    };

public:
    BinaryBoolean()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_a = modifier.addInput<bool>("A");
        in_b = modifier.addInput<bool>("B");

        out = modifier.addOutput<bool>("A OP B");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        std::map<std::string, Operation> operations
        {
            {"A AND B", Operation::AND},
            {"A OR B", Operation::OR},
            {"A NAND B", Operation::NAND},
            {"A NOR B", Operation::NOR},
            {"A XOR B", Operation::XOR}
        };
        params.addParameter(param::ParameterFactory::declareParameterSet("operation",
                                                                         operations,
                                                                         Operation::AND),
                            operation_);
    }

    void process() override
    {
        bool a = msg::getValue<bool>(in_a);
        bool b = msg::getValue<bool>(in_b);

        bool res;

        switch (operation_) {
        case Operation::AND:
            res = a && b;
            break;
        case Operation::OR:
            res = a || b;
            break;
        case Operation::NAND:
            res = !(a && b);
            break;
        case Operation::NOR:
            res = !(a || b);
            break;
        case Operation::XOR:
            res = a ^ b;
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


CSAPEX_REGISTER_CLASS(csapex::BinaryBoolean, csapex::Node)

