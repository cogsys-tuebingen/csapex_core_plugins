
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class OptimizationDummyVector : public Node
{
public:
    OptimizationDummyVector()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage, double>("params");
        in_x_ = modifier.addInput<double>("x");
        out_ = modifier.addOutput<double>("Output");
        out_grad_ = modifier.addOutput<GenericVectorMessage, double>("grad");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareOutputText("function"));
    }

    void process() override
    {
        double x = msg::getValue<double>(in_x_);
        auto params = msg::getMessage<GenericVectorMessage, double>(in_);

        std::shared_ptr<std::vector<double>> grad(new std::vector<double>);
        double i = 1;
        double res = 0;

        std::stringstream sstream;
        sstream << "f(x) = ";
        for (auto p : *params) {
            res += (x - i + p) * (x - i + p);
            if (i != 1) {
                sstream << " +" << std::endl;
            }
            sstream << "pow(" << x - i << " + "
                    << "p_" << i << ",2)  ";
            grad->push_back(2.0 * (x - i + p));
            ++i;
        }

        setParameter("function", sstream.str());
        msg::publish(out_, res);
        msg::publish<GenericVectorMessage, double>(out_grad_, grad);
    }

private:
    Input* in_;
    Input* in_x_;
    Output* out_;
    Output* out_grad_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::OptimizationDummyVector, csapex::Node)
