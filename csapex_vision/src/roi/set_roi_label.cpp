
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/roi_message.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_value_message.hpp>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class SetRoiLabel : public Node
{
public:
    SetRoiLabel()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_roi_ = modifier.addInput<RoiMessage>("ROI");
        in_value_ = modifier.addInput<AnyMessage>("value");
        out_ = modifier.addOutput<RoiMessage>("grown ROI");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        RoiMessage::ConstPtr roi = msg::getMessage<RoiMessage>(in_roi_);
        RoiMessage::Ptr out(new RoiMessage);

        std::stringstream label;

        if(msg::isValue<std::string>(in_value_)) {
            label << msg::getValue<std::string>(in_value_);

        } else if(msg::isValue<int>(in_value_)) {
            label << msg::getValue<int>(in_value_);

        } else if(msg::isValue<double>(in_value_)) {
            label << msg::getValue<double>(in_value_);

        } else {
            throw std::logic_error("unsupported value, supported are string, int, double");
        }

        out->value = roi->value;
        out->value.setLabel(label.str());
        msg::publish(out_, out);
    }

private:
    Input* in_roi_;
    Input* in_value_;
    Output* out_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::SetRoiLabel, csapex::Node)

