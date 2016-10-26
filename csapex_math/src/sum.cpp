
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

/// SYSTEM
#include <algorithm>
#include <numeric>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class Sum : public Node
{
public:
    Sum()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage>("Input");
        out_= modifier.addOutput<double>("Reduced");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        GenericVectorMessage::ConstPtr vec = msg::getMessage<GenericVectorMessage>(in_);
        if(std::dynamic_pointer_cast<GenericValueMessage<double>>(vec->nestedType())) {
            acc<double>(vec);
        } else if(std::dynamic_pointer_cast<GenericValueMessage<int>>(vec->nestedType())) {
            acc<int>(vec);
        }
    }

    template <typename T>
    void acc(GenericVectorMessage::ConstPtr vec )
    {
        double sum = 0;
        for(std::size_t i = 0, n = vec->nestedValueCount(); i < n; ++i) {
            if(auto pval = std::dynamic_pointer_cast<GenericValueMessage<T> const>(vec->nestedValue(i))) {
                sum += pval->value;
            }
        }

        msg::publish(out_, sum);
    }

private:
    Input* in_;
    Output* out_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::Sum, csapex::Node)

