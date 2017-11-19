
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


class GenericVectorMessageGetExtremum : public Node
{
public:
    GenericVectorMessageGetExtremum()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage>("Input");
        out_= modifier.addOutput<AnyMessage>("Extremum");
        out_id_ = modifier.addOutput<int>("Index");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareBool("publish_maximum",true),
                            publish_max_);
    }

    void process() override
    {
        GenericVectorMessage::ConstPtr vec = msg::getMessage<GenericVectorMessage>(in_);
        if(std::dynamic_pointer_cast<GenericValueMessage<double>>(vec->nestedType())) {
            findExtremum<double>(vec);
        } else if(std::dynamic_pointer_cast<GenericValueMessage<int>>(vec->nestedType())) {
            findExtremum<int>(vec);
        }
    }

    template <typename T>
    void findExtremum(GenericVectorMessage::ConstPtr vec )
    {
        int id = -1;
        T ext;
        if(publish_max_){
           ext = std::numeric_limits<T>::min();
        } else {
           ext = std::numeric_limits<T>::max();
        }
        for(std::size_t i = 0, n = vec->nestedValueCount(); i < n; ++i) {
            if(auto pval = std::dynamic_pointer_cast<GenericValueMessage<T> const>(vec->nestedValue(i))) {
                if(publish_max_){
                    if( pval->value > ext){
                        id = i;
                        ext = pval->value;
                    }
                }
                else{
                    if( pval->value < ext){
                        id = i;
                        ext = pval->value;
                    }
                }
            }
        }

        msg::publish(out_, ext);
        msg::publish(out_id_, id);
    }

private:
    Input* in_;
    Output* out_;
    Output* out_id_;
    bool publish_max_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::GenericVectorMessageGetExtremum, csapex::Node)

