/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

using namespace csapex::connection_types;

namespace csapex {

class LabeledScanToScan : public Node
{
public:
    LabeledScanToScan()
    {
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        in_ = node_modifier.addInput<LabeledScanMessage>("Labeled Scan");
        out_ = node_modifier.addOutput<ScanMessage>("Scan");
    }
    virtual void process() override
    {
        LabeledScanMessage::ConstPtr lmsg = msg::getMessage<LabeledScanMessage>(in_);

        ScanMessage::Ptr msg(new ScanMessage);
        msg->value = lmsg->value;

        msg::publish(out_, msg);
    }

private:
    Input* in_;
    Output* out_;
};
}

CSAPEX_REGISTER_CLASS(csapex::LabeledScanToScan, csapex::Node)
