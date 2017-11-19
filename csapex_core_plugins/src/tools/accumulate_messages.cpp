
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <deque>
using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class AccumulateMessages : public Node
{
public:
    AccumulateMessages()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Input");
        out_ = modifier.addOutput<connection_types::GenericVectorMessage>("vector of input");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareRange("number_of_messages",
                                                                  param::ParameterDescription("Number of messages to buffer. "
                                                                                              "If -1 vector is growing, n > 0 fixed size."),
                                                                  -1,100,10,1),
                            vector_size_);
    }

    void process() override
    {
//        AnyMessage::ConstPtr in_msg = msg::getMessage<AnyMessage>(in_);
        TokenData::ConstPtr in_msg = msg::getMessage<TokenData>(in_);

        buffer_.push_back(in_msg);

        if(vector_size_ >= 0){
            while(buffer_.size() > (std::size_t)vector_size_){
                buffer_.pop_front();
            }
        }

//        std::shared_ptr<std::vector<AnyMessage>> out(new std::vector<AnyMessage>);
        connection_types::GenericVectorMessage::Ptr result(GenericVectorMessage::make(in_msg));
        for(auto val : buffer_) {
            result->addNestedValue(val);
        }

        msg::publish(out_, result);

    }

private:
    Input* in_;
    Output* out_;
    int vector_size_;
    std::deque<TokenData::ConstPtr> buffer_;


};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::AccumulateMessages, csapex::Node)

