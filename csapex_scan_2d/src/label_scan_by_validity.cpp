/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex_scan_2d/scan_message.h>

using namespace csapex::connection_types;

namespace csapex
{
class LabelScanByValidity : public Node
{
public:
    LabelScanByValidity()
    {
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(param::factory::declareValue("label/invalid", -1), label_invalid_);
        parameters.addParameter(param::factory::declareValue("label/valid", 1), label_valid_);
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        in_ = node_modifier.addInput<LabeledScanMessage>("Labeled Scan");
        out_ = node_modifier.addOutput<LabeledScanMessage>("Re-Labeled Scan");
    }
    virtual void process() override
    {
        LabeledScanMessage::ConstPtr in_lmsg = msg::getMessage<LabeledScanMessage>(in_);
        LabeledScanMessage::Ptr out_lmsg(new LabeledScanMessage);
        out_lmsg->value = in_lmsg->value;

        lib_laser_processing::LabeledScan& lscan = out_lmsg->value;
        const std::size_t size = lscan.rays.size();
        for (std::size_t i = 0; i < size; ++i) {
            if (lscan.rays.at(i).invalid()) {
                lscan.labels.at(i) = label_invalid_;
            } else {
                lscan.labels.at(i) = label_valid_;
            }
        }
        msg::publish(out_, out_lmsg);
    }

private:
    Input* in_;
    Output* out_;

    int label_invalid_;
    int label_valid_;
};
}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::LabelScanByValidity, csapex::Node)
