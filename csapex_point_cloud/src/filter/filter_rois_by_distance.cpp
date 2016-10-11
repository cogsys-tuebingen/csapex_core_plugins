#include "filter_rois_by_distance.h"

#include <csapex/msg/io.h>
#include <csapex/msg/output.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>

CSAPEX_REGISTER_CLASS(csapex::FilterROIsByDistance, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

void FilterROIsByDistance::setupParameters(csapex::Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareParameterSet<int>("method",
                                                                              {{"mean", MEAN}, {"median", MEDIAN}},
                                                                              MEAN),
                            method_);
    parameters.addParameter(param::ParameterFactory::declareInterval("distance", -100.0, 100.0, 0.5, 5.0, 0.01),
                            distance_);
    parameters.addParameter(param::ParameterFactory::declareRange("min_point_count", 0, 100000, 0, 1),
                            min_point_count_);
    parameters.addParameter(param::ParameterFactory::declareBool("remove invalid", true),
                            remove_);
}

void FilterROIsByDistance::setup(csapex::NodeModifier& node_modifier)
{
    in_cloud_ = node_modifier.addInput<PointCloudMessage>("point cloud");
    in_rois_  = node_modifier.addInput<GenericVectorMessage, RoiMessage>("rois");
    out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("filtered rois");
}

void FilterROIsByDistance::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<FilterROIsByDistance>(this, msg), msg->value);
}

namespace
{
    template<typename PointT>
    inline bool check_filter(const RoiMessage &roi,
                             const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                             const std::pair<double, double>& distance,
                             int min_point_count,
                             FilterROIsByDistance::Method method)
    {
        cv::Rect rect = roi.value.rect() & cv::Rect(0, 0, cloud->width, cloud->height);

        if (method == FilterROIsByDistance::Method::MEAN)
        {
            std::size_t valid_points = 0;
            double      mean_depth = 0.0;

            for(int i = rect.y ; (i < rect.y + rect.height); ++i) {
                for(int j = rect.x ; (j < rect.x + rect.width); ++j) {
                    const PointT& p = cloud->at(j, i);
                    if(std::isnan(p.x) ||
                            std::isnan(p.y) ||
                                std::isnan(p.z))
                        continue;

                    if(p.x == 0.f &&
                            p.y == 0.f &&
                                p.z == 0.f)
                        continue;

                    mean_depth += p.x;
                    ++valid_points;
                }
            }

            mean_depth /= fmax(1, valid_points);
            bool constraints_apply = (int) valid_points > min_point_count &&
                                           mean_depth >= distance.first &&
                                           mean_depth <= distance.second;

            return constraints_apply;
        }
        else if (method == FilterROIsByDistance::Method::MEDIAN)
        {
            std::vector<double> depths;

            for(int i = rect.y ; (i < rect.y + rect.height); ++i) {
                for(int j = rect.x ; (j < rect.x + rect.width); ++j) {
                    const PointT& p = cloud->at(j, i);
                    if(std::isnan(p.x) ||
                            std::isnan(p.y) ||
                                std::isnan(p.z))
                        continue;

                    if(p.x == 0.f &&
                            p.y == 0.f &&
                                p.z == 0.f)
                        continue;

                    depths.push_back(p.x);
                }
            }

            if (depths.empty())
                return false;
            else
            {
                std::sort(depths.begin(), depths.end());
                double median_depth = depths[depths.size() / 2];
                bool constraints_apply = (int) depths.size() > min_point_count &&
                                               median_depth >= distance.first &&
                                               median_depth <= distance.second;

                return constraints_apply;
            }
        }
        else
            throw std::runtime_error("Invalid method type!");
    }
}

template<typename PointT>
void FilterROIsByDistance::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    std::shared_ptr<std::vector<RoiMessage> const> rois_msg = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    std::shared_ptr<std::vector<RoiMessage>> out_rois = std::make_shared<std::vector<RoiMessage>>();
    for (const RoiMessage& msg : *rois_msg)
    {
        bool valid = check_filter<PointT>(msg, cloud, distance_, min_point_count_, static_cast<Method>(method_));
        if(!remove_) {
            out_rois->push_back(msg);
            if(!valid) {
                out_rois->back().value.setClassification(2);
                out_rois->back().value.setColor(cv::Scalar(255, 255, 0));
            }
        } else {
            if(valid)
                out_rois->push_back(msg);
        }
    }

    msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, out_rois);
}
