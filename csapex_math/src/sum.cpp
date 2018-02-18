
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>
#include <csapex/model/token.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/variadic_io.h>

/// SYSTEM
#include <algorithm>
#include <numeric>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class Sum : public Node, public VariadicInputs
{
public:
    Sum()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        setupVariadic(modifier);

        out_= modifier.addOutput<double>("Sum");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        setupVariadicParameters(params);
    }

    virtual Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override
    {
        return VariadicInputs::createVariadicInput(multi_type::make<
                                                       GenericVectorMessage,
                                                       GenericValueMessage<int>,
                                                       GenericValueMessage<double>
                                                   >(),
                                                   label.empty() ? "Vector<Number> or Number" : label,
                                                   getVariadicInputCount() == 0 ? false : true);
    }

    void process() override
    {
        std::size_t num_inputs = VariadicInputs::getVariadicInputCount();

        sum = 0;

        for(std::size_t i_inputs = 0; i_inputs < num_inputs; ++i_inputs){
            addInput(VariadicInputs::getVariadicInput(i_inputs));
        }

        InputPtr in = VariadicInputs::getVariadicInput(0);
        if(msg::isValue<int>(in.get())) {
            int out = (int) sum;
            msg::publish(out_, out);

        } else if(msg::isValue<double>(in.get())) {
            msg::publish(out_, sum);

        } else {
            // must be vector
            GenericVectorMessageConstPtr vec = msg::getMessage<GenericVectorMessage>(in.get());
            if(std::dynamic_pointer_cast<connection_types::GenericValueMessage<int> const>(vec->nestedType())) {
                int out = (int) sum;
                msg::publish(out_, out);
            } else {
                msg::publish(out_, sum);
            }
        }

        msg::publish(out_, sum);
    }

    void addInput(const InputPtr& in)
    {
        if(msg::isValue<int>(in.get())) {
            sum += msg::getValue<int>(in.get());

        } else if(msg::isValue<double>(in.get())) {
            sum += msg::getValue<double>(in.get());

        } else {
            // must be vector
            GenericVectorMessageConstPtr vec = msg::getMessage<GenericVectorMessage>(in.get());
            if(std::dynamic_pointer_cast<GenericValueMessage<double>>(vec->nestedType())) {
                acc<double>(vec);
            } else if(std::dynamic_pointer_cast<GenericValueMessage<int>>(vec->nestedType())) {
                acc<int>(vec);
            }
        }
    }

    template <typename T>
    void acc(GenericVectorMessage::ConstPtr vec )
    {
        for(std::size_t i = 0, n = vec->nestedValueCount(); i < n; ++i) {
            if(auto pval = std::dynamic_pointer_cast<GenericValueMessage<T> const>(vec->nestedValue(i))) {
                sum += pval->value;
            }
        }
    }

private:
    Output* out_;

    double sum;
};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::Sum, csapex::Node)

