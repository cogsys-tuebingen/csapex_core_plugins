/// HEADER
#include "cloud_labeler.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::CloudLabeler, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

CloudLabeler::CloudLabeler()
{

}

void CloudLabeler::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("submit"), [this](param::Parameter*){
        done_request();
    });
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("drop"), [this](param::Parameter*){
        result_.reset();
        done();
    });

    std::function<void(csapex::param::Parameter*)> refresh = std::bind(&CloudLabeler::refresh, this);

    double d = 10.0;

    addParameter(csapex::param::ParameterFactory::declareRange("label",
                                                       csapex::param::ParameterDescription("The label to be assigned to the selected points"),
                                                       0, 9, 0, 1));
    addParameter(csapex::param::ParameterFactory::declareRange("radius",
                                                       csapex::param::ParameterDescription("The radius around the cursor to label in"),
                                                       0.02, 1.0, 0.05, 0.001));

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/r", 0.01, 20.0, 10.0, 0.01), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/theta", 0., M_PI, M_PI / 2, 0.001), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/phi", -M_PI, M_PI, 0., 0.001), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/dx", -d, d, 0., 0.01), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/dy", -d, d, 0., 0.01), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~view/dz", -d, d, 0., 0.01), refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~size/width", 10, 4096, 400, 1), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~size/height", 10, 4096, 400, 1), refresh);

    csapex::param::Parameter::Ptr sync = csapex::param::ParameterFactory::declareBool("~size/out/sync", true);
    parameters.addParameter(sync, refresh);
    std::function<bool()> notsync = [sync]() { return !sync->as<bool>(); };
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange("~size/out/width", 10, 1024, 400, 1), notsync, refresh);
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange("~size/out/height", 10, 1024, 400, 1), notsync, refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareColorParameter("color/background", 255, 255, 255), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareColorParameter("color/grid", 0, 0, 0), refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareBool("show axes", false), refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~grid/size", 1, 30, 10, 1), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("~grid/resolution", 0.1, 10.0, 1.0, 0.1), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("~grid/xy", true), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("~grid/yz", false), refresh);
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("~grid/xz", false), refresh);

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("point/size", 1., 30., 5., 0.1), refresh);
}

void CloudLabeler::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    output_ = node_modifier.addOutput<PointCloudMessage>("Labeled Cloud");
}

void CloudLabeler::beginProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters)
{
    PointCloudMessage::ConstPtr msg = msg::getMessage<PointCloudMessage>(input_);

    {
        std::unique_lock<std::mutex> lock(message_mutex_);
        message_ = msg;
        result_.reset(new PointCloudMessage(msg->frame_id, msg->stamp_micro_seconds));
    }

    display_request();
}

void CloudLabeler::finishProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters)
{
    if(msg::isConnected(output_)) {
        if(result_) {
            msg::publish(output_, result_);
        }
    }
}

connection_types::PointCloudMessage::ConstPtr CloudLabeler::getMessage() const
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    return message_;
}

bool CloudLabeler::isOutputConnected() const
{
    return msg::isConnected(output_);
}

void CloudLabeler::refresh()
{
    refresh_request();
}

void CloudLabeler::setResult(const pcl::PointCloud<pcl::PointXYZL>::Ptr &labeled)
{
    result_->value = labeled;

    done();
}
