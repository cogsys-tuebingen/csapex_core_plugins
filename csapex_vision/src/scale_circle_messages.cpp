
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_opencv/circle_message.h>
using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class ScaleCircleMessages : public Node
{
public:
    ScaleCircleMessages()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
         in_ = modifier.addMultiInput<CircleMessage, GenericVectorMessage>("Input");
         out_ = modifier.addOutput<AnyMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(csapex::param::factory::declareRange("percent x", 1.0, 400.0, 100.0, 1.0),
                            [this](param::Parameter* p){
            scales_[0] = p->as<double>()/100.0;
        });
        params.addParameter(csapex::param::factory::declareRange("percent y", 1.0, 400.0, 100.0, 1.0),
                            [this](param::Parameter* p){
            scales_[1] = p->as<double>()/100.0;
        });
    }

    void process() override
    {
        if (msg::isMessage<CircleMessage>(in_)) {
            CircleMessage::ConstPtr c_msg = msg::getMessage<CircleMessage>(in_);
            CircleMessage::Ptr out(new CircleMessage);
            scale(*c_msg, *out);
            msg::publish(out_, out);

        } else {
            GenericVectorMessage::ConstPtr message = msg::getMessage<GenericVectorMessage>(in_);
            apex_assert(std::dynamic_pointer_cast<CircleMessage>(message->nestedType()));
            std::size_t n_circles = message->nestedValueCount();
            std::shared_ptr<std::vector<CircleMessage>> out(new std::vector<CircleMessage>);

            for (std::size_t i = 0; i < n_circles; ++i) {
                CircleMessage::ConstPtr circle = std::dynamic_pointer_cast<CircleMessage const>(message->nestedValue(i));
                CircleMessage tmp;
                scale(*circle, tmp);
                out->push_back(tmp);
            }
            msg::publish<GenericVectorMessage, CircleMessage>(out_, out);
        }
    }

    void scale(const CircleMessage& in, CircleMessage& out)
    {
        out.frame_id = in.frame_id;
        out.stamp_micro_seconds = in.stamp_micro_seconds;
        out.center_x = scales_[0] * in.center_x;
        out.center_y = scales_[1] * in.center_y;
        out.radius = 0.5*(scales_[0] + scales_[1]) * in.radius;
        out.id = in.id;
    }

private:
    Input* in_;
    Output* out_;
    std::array<double,2> scales_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::ScaleCircleMessages, csapex::Node)

