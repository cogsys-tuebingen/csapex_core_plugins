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
            -1, -w, 1, w
        };

        PointT const* in_pt = &in.points[w];
        PointT* out_pt = &out.points[w];
        for(int row = 1, col = 0;
            row < h - 1;
            ++in_pt, ++out_pt) {

            // don't look at border
            if(col > 0 && col < w - 1) {

                double min_diff = std::numeric_limits<double>::infinity();
                for(std::size_t i = 0; i < 4; ++i) {
                    PointT const* neighbor = in_pt - offset[i];
                    double diff = std::abs(neighbor->z - in_pt->z);
                    if(diff < min_diff) {
                        min_diff = diff;
                    }
                }

                if(min_diff > max_distance_) {
                    out_pt->x = std::numeric_limits<double>::quiet_NaN ();
                    out_pt->y = std::numeric_limits<double>::quiet_NaN ();
                    out_pt->z = std::numeric_limits<double>::quiet_NaN ();
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
};

}

CSAPEX_REGISTER_CLASS(csapex::EdgeFilter, csapex::Node)

#endif // EDGE_FILTER_H
