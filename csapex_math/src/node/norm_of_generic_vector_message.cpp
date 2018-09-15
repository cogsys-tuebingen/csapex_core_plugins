
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


class NormOfGenericVectorMessage : public Node
{
public:
    NormOfGenericVectorMessage()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage>("Input");
        out_ = modifier.addOutput<AnyMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        GenericVectorMessage::ConstPtr vec = msg::getMessage<GenericVectorMessage>(in_);
        if(std::dynamic_pointer_cast<GenericValueMessage<double>>(vec->nestedType())) {
           norm<double>(vec);
        } else if(std::dynamic_pointer_cast<GenericValueMessage<int>>(vec->nestedType())) {
            norm<int>(vec);
        }
    }

    template <typename T>
    void norm(GenericVectorMessage::ConstPtr vec)
    {
        std::size_t n = vec->nestedValueCount();
        T norm_val = 0;

        for(std::size_t i = 0; i < n; ++i) {
            if(auto pval = std::dynamic_pointer_cast<GenericValueMessage<T> const>(vec->nestedValue(i))) {
                norm_val += std::pow(pval->value,2);
            }
        }
        norm_val = std::sqrt(norm_val);
        msg::publish(out_, norm_val);
    }

private:
    Input* in_;
    Output* out_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::NormOfGenericVectorMessage, csapex::Node)

