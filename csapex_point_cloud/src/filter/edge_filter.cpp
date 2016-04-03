#ifndef EDGE_FILTER_H
#define EDGE_FILTER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>

/// SYSTEM
#include <pcl/filters/radius_outlier_removal.h>

using namespace csapex::connection_types;

namespace {

template <typename PointT>
double distance(const PointT& a)
{
    auto dx = a.x;
    auto dy = a.y;
    auto dz = a.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
}

namespace csapex
{

class EdgeFilter : public Node
{
public:
    EdgeFilter()
    {

    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<PointCloudMessage>("cloud");
        output_ = node_modifier.addOutput<PointCloudMessage>("filtered cloud");
    }

    void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(param::ParameterFactory::declareRange
                                ("max distance",
                                 0.0, 0.1, 0.05, 0.0001),
                                max_distance_);
        parameters.addParameter(param::ParameterFactory::declareRange
                                ("offset",
                                 1, 16, 1, 1),
                                offset_);
        parameters.addParameter(param::ParameterFactory::declareRange
                                ("min height",
                                 -5.0, 5.0, 0.0, 0.01),
                                min_height_);

        parameters.addParameter(param::ParameterFactory::declareRange
                                ("distance factor",
                                 0.0, 1.0, 1.0, 0.001),
                                distance_factor_);

    }

    virtual void process() override
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

        boost::apply_visitor (PointCloudMessage::Dispatch<EdgeFilter>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        out->header = cloud->header;

        bool dense = cloud->width > 1 && cloud->height > 1;
        apex_assert(dense);
        apex_assert(cloud->width * cloud->height == cloud->points.size());

        filter<PointT>(*cloud, *out);

        PointCloudMessage::Ptr out_msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        out_msg->value = out;
        msg::publish(output_, out_msg);
    }

    template <class PointT>
    void filter(const pcl::PointCloud<PointT>& in,
                pcl::PointCloud<PointT>& out)
    {
        out = in;

        if(max_distance_ == 0.0) {
            return;
        }

        int w = in.width;
        int h = in.height;

        int offset[] = {
            -offset_, -w*offset_, offset_, w*offset_
        };

        PointT const* in_pt = &in.points[w * offset_ + offset_];
        PointT* out_pt = &out.points[w * offset_ + offset_];
        for(int row = offset_, col = 0;
            row < h - offset_;
            ++in_pt, ++out_pt) {

            // don't filter points below min_height
            if(in_pt->z >= min_height_) {
                // don't look at border
                if(col > offset_ && col < w - offset_) {

                    //                    double factor = distance_factor_ * distanceSqr(mean2);

                    double min_diff = std::numeric_limits<double>::infinity();
                    for(std::size_t i = 0; i < 4; ++i) {
                        PointT const* neighbor = in_pt - offset[i];

                        double d = distance(*in_pt);
                        double factor = distance_factor_ == 0 ? 1.0 : distance_factor_ * d;
                        double diff = factor * std::abs(distance(*neighbor) - d);
                        if(diff < min_diff) {
                            min_diff = diff;
                        }
                    }

                    if(min_diff > max_distance_) {
                        out_pt->x = std::numeric_limits<float>::quiet_NaN ();
                        out_pt->y = std::numeric_limits<float>::quiet_NaN ();
                        out_pt->z = std::numeric_limits<float>::quiet_NaN ();
                    }
                }
            }

            if(++col >= w) {
                col = 0;
                ++row;
            }
        }
    }

private:
    Input*  input_;
    Output* output_;

    double max_distance_;
    int offset_;

    double min_height_;
    double distance_factor_;
};

}

CSAPEX_REGISTER_CLASS(csapex::EdgeFilter, csapex::Node)

#endif // EDGE_FILTER_H
