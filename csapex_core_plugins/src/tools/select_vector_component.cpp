/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/serialization/io/csapex_io.h>


using namespace csapex::connection_types;
namespace csapex
{
class CSAPEX_EXPORT_PLUGIN SelectVectorComponent : public Node
{
public:
    SelectVectorComponent(): comp_(0){}

    void setup(csapex::NodeModifier& node_modifier) override
    {
        in_ = node_modifier.addInput<GenericVectorMessage, AnyMessage>("input");
        out_ = node_modifier.addOutput<AnyMessage>("output");

    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {

        std::function<void(param::Parameter* p)> cb = [this](param::Parameter* p){
            apex_assert(p->as<int>() >= 0);
            comp_ = p->as<int>();
        };

        parameters.addParameter(param::factory::declareValue(
                                    "component", 0),
                                cb);
    }


    void process()
    {

        auto in_msg = msg::getMessage<connection_types::GenericVectorMessage>(in_);

        if(in_msg->nestedValueCount() > comp_){

            msg::publish(out_, in_msg->nestedValue(comp_));
        }

    }




private:

    Input* in_;

    Output* out_;

    std::size_t comp_;


};


CSAPEX_REGISTER_CLASS(csapex::SelectVectorComponent, csapex::Node)}



