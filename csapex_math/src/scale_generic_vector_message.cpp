
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/generic_vector_message.hpp>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class ScaleGenericVectorMessage : public Node
{
public:
    ScaleGenericVectorMessage()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage>("Input");
        out_ = modifier.addOutput<GenericVectorMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareValue("scale",1.0),
                            scale_);
    }

    void process() override
    {
        GenericVectorMessage::ConstPtr vec = msg::getMessage<GenericVectorMessage>(in_);
        if(std::dynamic_pointer_cast<GenericValueMessage<double>>(vec->nestedType())) {
           scale<double>(vec);
        } else if(std::dynamic_pointer_cast<GenericValueMessage<int>>(vec->nestedType())) {
            scale<int>(vec);
        }

    }

    template <typename T>
    void scale(GenericVectorMessage::ConstPtr vec)
    {
        std::size_t n = vec->nestedValueCount();
        std::shared_ptr<std::vector<T>> scaled(new std::vector<T>);
        scaled->resize(n);
        for(std::size_t i = 0; i < n; ++i) {
            if(auto pval = std::dynamic_pointer_cast<GenericValueMessage<T> const>(vec->nestedValue(i))) {
                scaled->at(i) = scale_ * pval->value;
            }
        }
        msg::publish<GenericVectorMessage, T>(out_, scaled);
    }

private:
    Input* in_;
    Output* out_;
    double scale_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::ScaleGenericVectorMessage, csapex::Node)

