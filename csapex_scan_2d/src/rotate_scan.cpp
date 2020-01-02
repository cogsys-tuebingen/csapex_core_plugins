
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_math/param/factory.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex_scan_2d/scan_message.h>

namespace csapex
{
using namespace csapex;
using namespace csapex::connection_types;
using namespace lib_laser_processing;

const double _2_M_PI = 2.0 * M_PI;
const double _1_2_M_PI = 1.0 / _2_M_PI;

inline double normalize(const double angle)
{
    return angle - _2_M_PI * floor((angle + M_PI) * _1_2_M_PI);
}

class RotateScan : public csapex::Node
{
public:
    RotateScan() = default;

    void setupParameters(Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareAngle("rotation", 0.0), angle_);
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addMultiInput<ScanMessage, LabeledScanMessage>("Scan");
        output_ = node_modifier.addOutput<AnyMessage>("Rotated Scan");
    }

    void process() override
    {
        if (msg::isMessage<LabeledScanMessage>(input_)) {
            LabeledScanMessage::ConstPtr scan_msg = msg::getMessage<LabeledScanMessage>(input_);
            LabeledScanMessage::Ptr out_scan_msg(new LabeledScanMessage(*scan_msg));
            out_scan_msg->angle_min = normalize(out_scan_msg->angle_min + angle_);
            out_scan_msg->angle_max = normalize(out_scan_msg->angle_max + angle_);
            doProcess<LabeledScan>(out_scan_msg->value);
            msg::publish(output_, out_scan_msg);
        } else if (msg::isMessage<ScanMessage>(input_)) {
            ScanMessage::ConstPtr scan_msg = msg::getMessage<ScanMessage>(input_);
            ScanMessage::Ptr out_scan_msg(new ScanMessage(*scan_msg));
            out_scan_msg->angle_min = normalize(out_scan_msg->angle_min + angle_);
            out_scan_msg->angle_max = normalize(out_scan_msg->angle_max + angle_);
            doProcess<Scan>(out_scan_msg->value);
            msg::publish(output_, out_scan_msg);
        } else {
            throw std::runtime_error("invalid input type");
        }
    }

    template <typename ScanType>
    void doProcess(ScanType& scan)
    {
        for (auto& r : scan.rays) {
            const double angle = r.yaw() + angle_;
            r = LaserBeam(angle, r.range());
        }
    }

private:
    Input* input_;
    Output* output_;

    double angle_;
};
}  // namespace csapex
CSAPEX_REGISTER_CLASS(csapex::RotateScan, csapex::Node)
