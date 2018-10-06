/// HEADER
#include "filter.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_scan_2d/scan_message.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace lib_laser_processing;

CSAPEX_REGISTER_CLASS(csapex::ScanFilter, csapex::Node)

ScanFilter::ScanFilter()
{
}

void ScanFilter::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("max angle", 0.0, M_PI, M_PI, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange("min angle", -M_PI, 0.0, -M_PI, 0.1));

    parameters.addParameter(csapex::param::factory::declareRange("max range", 0.0, 30.0, 30.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange("min range", 0.0, 30.0, 0.0, 0.1));
}

void ScanFilter::process()
{
    ScanMessage::ConstPtr scan_msg = msg::getMessage<ScanMessage>(input_);
    ScanMessage::Ptr filtered_scan_msg(new ScanMessage);
    filtered_scan_msg->value = scan_msg->value;
    filtered_scan_msg->frame_id = scan_msg->frame_id;
    filtered_scan_msg->stamp_micro_seconds = scan_msg->stamp_micro_seconds;

    const Scan& scan = scan_msg->value;
    Scan& filtered_scan = filtered_scan_msg->value;
    filtered_scan.rays.clear();

    double max_range = readParameter<double>("max range");
    double min_range = readParameter<double>("min range");
    double max_angle = readParameter<double>("max angle");
    double min_angle = readParameter<double>("min angle");

    double angle = scan.angle_min;
    for (std::vector<LaserBeam>::const_iterator it = scan.rays.begin(); it != scan.rays.end(); ++it, angle += scan.angle_increment) {
        const LaserBeam& ray = *it;
        if (angle >= min_angle && angle <= max_angle && ray.range() > min_range && ray.range() < max_range) {
            filtered_scan.rays.push_back(ray);
        } else {
            filtered_scan.rays.push_back(LaserBeam(ray.yaw(), 0.0f));
        }
    }

    msg::publish(output_, filtered_scan_msg);
}

void ScanFilter::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<ScanMessage>("Scan");
    output_ = node_modifier.addOutput<ScanMessage>("Filtered Scan");
}
