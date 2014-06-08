/// HEADER
#include "model_to_marker.h"

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_core_plugins/vector_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <geometry_msgs/Point.h>

CSAPEX_REGISTER_CLASS(csapex::ModelToMarker, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ModelToMarker::ModelToMarker()
{
    //addTag(Tag::get("PointCloud"));
}

void ModelToMarker::process()
{
    boost::shared_ptr<std::vector<ModelMessage> const> models = input_->getMessage<GenericVectorMessage, ModelMessage>();

    const std::vector<int>& color = param<std::vector<int> >("color/marker");

    if(param<bool>("publish marker")) {
        visualization_msgs::MarkerArray::Ptr marker_array(new visualization_msgs::MarkerArray);
        int marker_id = 0;
        for (std::vector<ModelMessage>::const_iterator it = models->begin(); it != models->end(); it++) {

            visualization_msgs::Marker::Ptr marker(new visualization_msgs::Marker);
            generateMarker(*(it), marker, color);
            marker->id = marker_id;
            marker_id ++;
            marker_array->markers.push_back(*marker);

            publishText(*(it)); // publish the model as text for debuging
        }
        output_->publish<visualization_msgs::MarkerArray>(marker_array);
    }


}

void ModelToMarker::setup()
{
    input_ = modifier_->addInput<GenericVectorMessage, ModelMessage >("ModelMessages");
    output_ = modifier_->addOutput<visualization_msgs::MarkerArray>("Marker");
    output_text_ = modifier_->addOutput<std::string>("String");

    addParameter(param::ParameterFactory::declareBool("publish marker", true));
    addParameter(param::ParameterFactory::declareColorParameter("color/marker", 0xFF, 0xCC, 0x00));
}


void ModelToMarker::generateMarker(const ModelMessage model_message, const visualization_msgs::Marker::Ptr marker, std::vector<int> color)
{
    //visualization_msgs::MarkerArray::Ptr marker_array(new visualization_msgs::MarkerArray);
    //visualization_msgs::Marker           marker;


    marker->header.frame_id     = model_message.frame_id;
    marker->header.stamp        = ros::Time::now();
    marker->ns                  = "model";
    marker->id                  = 1;
    marker->action              = visualization_msgs::Marker::ADD;

    marker->color.a = 0.8;
    marker->color.r = color.at(0) / 255.0;;
    marker->color.g = color.at(1) / 255.0;;
    marker->color.b = color.at(2) / 255.0;;

    if (model_message.model_type == pcl::SACMODEL_SPHERE ) {
        marker->type                = visualization_msgs::Marker::SPHERE;

        marker->pose.position.x = model_message.coefficients->values.at(0);
        marker->pose.position.y = model_message.coefficients->values.at(1);
        marker->pose.position.z = model_message.coefficients->values.at(2);

        marker->pose.orientation.x  = 0.0;
        marker->pose.orientation.y  = 0.0;
        marker->pose.orientation.z  = 0.0;
        marker->pose.orientation.w  = 1.0;

        double min_scale = 0.001;
        double scale = model_message.coefficients->values.at(3) > min_scale ? model_message.coefficients->values.at(3) : min_scale;
        marker->scale.x = scale;
        marker->scale.y = scale;
        marker->scale.z = scale;



    } else if (model_message.model_type == pcl::SACMODEL_CONE ) {
        marker->type                = visualization_msgs::Marker::ARROW;

        geometry_msgs::Point apex;
        apex.x = model_message.coefficients->values.at(0);
        apex.y = model_message.coefficients->values.at(1);
        apex.z = model_message.coefficients->values.at(2);

        geometry_msgs::Point base;
        base.x = apex.x + model_message.coefficients->values.at(3);
        base.y = apex.y + model_message.coefficients->values.at(4);
        base.z = apex.z + model_message.coefficients->values.at(5);

        marker->points.push_back(base);
        marker->points.push_back(apex);
//        marker->pose.orientation.x  = 0.0;
//        marker->pose.orientation.y  = 0.0;
//        marker->pose.orientation.z  = 0.0;
//        marker->pose.orientation.w  = 1.0;

        // Normal vector of the cone
        double nx = model_message.coefficients->values.at(3);
        double ny = base.y = apex.y + model_message.coefficients->values.at(4);
        double nz = base.z = apex.z + model_message.coefficients->values.at(5);
        double len = sqrt( nx*nx + ny*ny + nz*nz);
        double opening_angle = model_message.coefficients->values.at(5);


        double scale = 1.0;
        marker->scale.x = scale;
        marker->scale.y = len * tan(opening_angle);
        marker->scale.z = 100;


    // 2D Circle
    } else if (model_message.model_type == pcl::SACMODEL_CIRCLE2D ) {
        marker->type                = visualization_msgs::Marker::CYLINDER;

        marker->pose.position.x = model_message.coefficients->values.at(0);
        marker->pose.position.y = model_message.coefficients->values.at(1);
        marker->pose.position.z = 0;

        marker->pose.orientation.x  = 0.0;
        marker->pose.orientation.y  = 0.0;
        marker->pose.orientation.z  = 0.0;
        marker->pose.orientation.w  = 1.0;

        marker->scale.x = model_message.coefficients->values.at(2);
        marker->scale.y = model_message.coefficients->values.at(2);
        marker->scale.z = model_message.probability;

    } else {
        printf("unknown Model!!");
    }

    // Todo:
    // Add plane and cylinder
    // Change to marker array so that a nicer cone can be displayed by a vector and a circle

}

void ModelToMarker::publishText(const ModelMessage model_message)
{
    // Publish the model as Text
    std::stringstream stringstream;
    stringstream << "Model Type: " << model_message.model_type;
    stringstream << " Frame: " << model_message.frame_id;
    for (std::size_t i1=0; i1 < model_message.coefficients->values.size(); i1++) {
        stringstream << " [" << i1 << "]=" << model_message.coefficients->values.at(i1);
    }
    stringstream << " Prob: " << model_message.probability;
    std::string text_msg = stringstream.str();
    output_text_->publishIntegral(text_msg);
}

