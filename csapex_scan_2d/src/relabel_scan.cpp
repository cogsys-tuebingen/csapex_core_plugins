/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

using namespace csapex::connection_types;

namespace csapex {

class RelabelScan : public Node
{
public:
    RelabelScan()
    {
    }

    void setupParameters(Parameterizable &parameters)
    {
        parameters.addParameter(param::ParameterFactory::declareValue("label/old", 0),
                                label_old_);
        parameters.addParameter(param::ParameterFactory::declareValue("label/new", -1),
                                label_new_);
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        in_ = node_modifier.addInput<LabeledScanMessage>("Labeled Scan");
        out_ = node_modifier.addOutput<LabeledScanMessage>("Re-Labeled Scan");
    }
    virtual void process() override
    {
        LabeledScanMessage::ConstPtr in_lmsg = msg::getMessage<LabeledScanMessage>(in_);
        LabeledScanMessage::Ptr      out_lmsg(new LabeledScanMessage);
        out_lmsg->value = in_lmsg->value;

        lib_laser_processing::LabeledScan &lscan = out_lmsg->value;
        const std::size_t size = lscan.rays.size();
        for(std::size_t i = 0 ; i < size ; ++i) {
            if(lscan.labels.at(i) == label_old_) {
                lscan.labels.at(i) = label_new_;
            }
        }
        msg::publish(out_, out_lmsg);
    }

private:
    Input* in_;
    Output* out_;

    int label_old_;
    int label_new_;

};
}

CSAPEX_REGISTER_CLASS(csapex::RelabelScan, csapex::Node)
