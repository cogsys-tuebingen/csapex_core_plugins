/// HEADER
#include "passthrough.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/mpl/for_each.hpp>
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif //__clang__
#include <pcl/filters/passthrough.h>
#if __clang__
#pragma clang diagnostic pop
#endif //__clang__
#include <boost/assign/std.hpp>

CSAPEX_REGISTER_CLASS(csapex::PassThrough, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PassThrough::PassThrough()
{
}

void PassThrough::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareInterval("interval", -100.0, 100.0, 0.0, 100.0, 0.01));
    parameters.addParameter(param::ParameterFactory::declareBool("keep organized", true));

    std::vector<std::string> field;
    field.push_back("x");
    parameters.addParameter(param::ParameterFactory::declareParameterStringSet("field", field), std::bind(&PassThrough::updateBorders, this));
}

void PassThrough::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");

    output_pos_ = node_modifier.addOutput<PointCloudMessage>("cropped PointCloud (+)");
    output_neg_ = node_modifier.addOutput<PointCloudMessage>("cropped PointCloud (-)");
}

void PassThrough::updateBorders()
{
    std::string field = readParameter<std::string>("field");
    param::IntervalParameter::Ptr interv = getParameter<param::IntervalParameter>("interval");

    if(field == "x" || field == "y" || field == "z") {
        interv->setInterval(-10.0, 10.0);
    } else {
        interv->setInterval(0.0, 256.0);
    }
}

void PassThrough::updateFields(const std::vector<std::string>& fields)
{
    if(fields.size() == fields_.size()) {
        return;
    }

    fields_ = fields;

    param::SetParameter::Ptr setp = std::dynamic_pointer_cast<param::SetParameter>(getParameter("field"));
    if(setp) {
        modifier_->setNoError();
        std::string old_field = readParameter<std::string>("field");
        setp->setSet(fields);
        setp->set(old_field);
    }
}

void PassThrough::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));

    boost::apply_visitor (PointCloudMessage::Dispatch<PassThrough>(this, msg), msg->value);
}

template <class PointT>
void PassThrough::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::vector<pcl::PCLPointField> fields;
    std::vector<std::string> field_names;
    pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

    for(size_t d = 0; d < fields.size (); ++d) {
        field_names.push_back(fields[d].name);
    }

    updateFields(field_names);


    // check available fields!
    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName (readParameter<std::string>("field"));

    param::IntervalParameter::Ptr interv = getParameter<param::IntervalParameter>("interval");
    pass.setFilterLimits (interv->lower<double>(), interv->upper<double>());
    pass.setInputCloud(cloud);
    pass.setKeepOrganized(readParameter<bool>("keep organized"));

    if(msg::isConnected(output_pos_)) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        msg->value = out;
        msg::publish(output_pos_, msg);
    }

    if(msg::isConnected(output_neg_)) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.setNegative(true);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        msg->value = out;
        msg::publish(output_neg_, msg);
    }

}
