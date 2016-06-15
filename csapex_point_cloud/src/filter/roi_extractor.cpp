#include "roi_extractor.hpp"

#include <csapex/msg/io.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_point_cloud/indeces_message.h>

#include <pcl/filters/crop_box.h>

CSAPEX_REGISTER_CLASS(csapex::ROIExtractor, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

ROIExtractor::ROIExtractor()
{

}

void ROIExtractor::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange<int>("outputs", 0, 10, 0, 1),
                            std::bind(&ROIExtractor::updateOutputs, this));

    param::ParameterPtr filter_param = param::ParameterFactory::declareBool("filter", false);
    parameters.addParameter(filter_param,
                            filter_);
    parameters.addConditionalParameter(param::ParameterFactory::declareRange("filter/class", -100, 100, 0, 1),
                                       [filter_param]() { return filter_param->as<bool>(); },
                                       filter_class_);

}

void ROIExtractor::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("Source PointCloud");
    input_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
    input_indices_ = node_modifier.addOptionalInput<GenericVectorMessage, pcl::PointIndices>("Indices");

    output_clouds_ = node_modifier.addOutput<VectorMessage, PointCloudMessage>("PointClouds");
}

void ROIExtractor::updateOutputs()
{
    std::size_t new_count = readParameter<int>("outputs");
    if (new_count != output_clouds_single_.size())
    {
        if (new_count > output_clouds_single_.size())
        {
            for (std::size_t i = output_clouds_single_.size(); i < new_count; ++i)
            {
                std::stringstream name;
                name << "Single Cloud " << i;
                output_clouds_single_.push_back(node_modifier_->addOutput<PointCloudMessage>(name.str()));
            }
        }
        else
        {
            for (int i = output_clouds_single_.size() - 1; i >= new_count; --i)
            {
                node_modifier_->removeOutput(msg::getUUID(output_clouds_single_[i]));
                output_clouds_single_.pop_back();
            }
        }
    }
}

void ROIExtractor::publish(VectorMessage::Ptr message)
{
    for (std::size_t i = 0; i < output_clouds_single_.size() && i < message->value.size(); ++i)
        msg::publish(output_clouds_single_[i], message->value[i]);

    msg::publish(output_clouds_, message);
}

void ROIExtractor::process()
{
    PointCloudMessage::ConstPtr pcl(msg::getMessage<PointCloudMessage>(input_cloud_));

    boost::apply_visitor(PointCloudMessage::Dispatch<ROIExtractor>(this, pcl), pcl->value);
}

template<typename PointT>
void ROIExtractor::extract_organized(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::shared_ptr<std::vector<RoiMessage> const> roi_vector(msg::getMessage<GenericVectorMessage, RoiMessage>(input_rois_));
    VectorMessage::Ptr out_vector(VectorMessage::make<PointCloudMessage>());

    if (!cloud->isOrganized())
        throw std::runtime_error("Cluster index list required for unorganized clouds");

    for (const auto& roi_msg : *roi_vector)
    {
        if (filter_)
            if (roi_msg.value.classification() != filter_class_)
                continue;

        cv::Rect region = roi_msg.value.rect() & cv::Rect(0, 0, cloud->width, cloud->height);

        typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>(region.width, region.height));
        result->header = cloud->header;
        result->is_dense = cloud->is_dense;
        for (int y = region.y; y < region.y + region.height; ++y)
            for (int x = region.x; x < region.x + region.width; ++x)
                (*result)(x - region.x, y - region.y) = (*cloud)(x, y);

        PointCloudMessage::Ptr out_msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        out_msg->value = result;
        out_vector->value.push_back(out_msg);
    }

    publish(out_vector);
}

template<typename PointT>
void ROIExtractor::extract_unorganized(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::shared_ptr<std::vector<RoiMessage> const> roi_vector(msg::getMessage<GenericVectorMessage, RoiMessage>(input_rois_));
    std::shared_ptr<std::vector<pcl::PointIndices> const> indices = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(input_indices_);
    VectorMessage::Ptr out_vector(VectorMessage::make<PointCloudMessage>());

    if (roi_vector->size() != indices->size())
        throw std::runtime_error("ROIs and Indices vectors do not have equal length");

    int idx = 0;
    for (const auto& roi_msg : *roi_vector)
    {
        if (filter_)
        {
            if (roi_msg.value.classification() != filter_class_)
            {
                ++idx;
                continue;
            }
        }

        /*
        cv::Point top_left = roi_msg->value.rect().tl();
        cv::Point bottom_right = roi_msg->value.rect().br();

        const PointT& tl = cloud->at(top_left.x, top_left.y);
        const PointT& br = cloud->at(bottom_right.x, bottom_right.y);

        Eigen::Vector4f min_pt(tl.x, tl.y, 0.f, 0.f);
        Eigen::Vector4f max_pt(br.x, br.y, 100.f, 0.f);

        std::cout << tl << " " << br << std::endl;

        pcl::CropBox<PointT> crop;
        crop.setInputCloud(cloud);
        crop.setMin(min_pt);
        crop.setMax(max_pt);
        typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>());
        crop.filter(*result);
        result->header = cloud->header;
        */

        typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>(*cloud, indices->at(idx).indices));
        result->header = cloud->header;
        result->is_dense = cloud->is_dense;

        PointCloudMessage::Ptr out_msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        out_msg->value = result;
        out_vector->value.push_back(out_msg);

        ++idx;
    }

    publish(out_vector);
}

template<typename PointT>
void ROIExtractor::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if (msg::hasMessage(input_indices_))
        extract_unorganized<PointT>(cloud);
    else
        extract_organized<PointT>(cloud);
}

