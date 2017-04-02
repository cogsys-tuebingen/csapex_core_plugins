/// HEADER
#include "plane_segmentation.h"

/// PROJECT
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/normals_message.h>
#include <csapex_opencv/cv_mat_message.h>

#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>


/// SYSTEM
#include <opencv2/opencv.hpp>

CSAPEX_REGISTER_CLASS(csapex::PlaneSegmentation, csapex::Node)

using namespace csapex;
using namespace connection_types;

void PlaneSegmentation::setup(NodeModifier &node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("input cloud");
    output_normals_ = node_modifier.addOutput<NormalsMessage>("output normals");
}

void PlaneSegmentation::setupParameters(Parameterizable &parameters)
{
}

void PlaneSegmentation::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<PlaneSegmentation>(this, msg), msg->value);
}

template<class PointT>
void PlaneSegmentation::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if(!cloud->isOrganized()) {
        throw std::runtime_error("This node only works with structured point clouds.");
    }


    const std::size_t height = cloud->height;
    const std::size_t width = cloud->width;
    const std::size_t size = height * width;

    if(height < 2 || width < 2) {
        throw std::runtime_error("The inserted point cloud is to small!");
    }

    cv::Mat points(height, width, CV_32FC3, cv::Scalar());
    cv::Mat mask(height, width, CV_8UC1, cv::Scalar(1));
    cv::Mat normals_mask(height, width, CV_8UC1, cv::Scalar(1));

    /// 1. we copy the point cloud into a cv mat for further processing
    ///    a mask is computed encapsuling the information is a point is invalid
    const PointT *cloud_ptr = cloud->points.data();
    cv::Vec3f *points_ptr = points.ptr<cv::Vec3f>();
    uchar     *mask_ptr = mask.ptr<uchar>();

    auto invalid = [](const PointT &p)
    {
        const bool inf = std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z);
        const bool nan = std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
        return inf || nan;
    };

    for(std::size_t i = 0 ; i < size ; ++i) {
        const PointT &cloud_point = cloud_ptr[i];
        cv::Vec3f &point = points_ptr[i];
        point[0] = cloud_point.x;
        point[1] = cloud_point.y;
        point[2] = cloud_point.z;
        if(invalid(cloud_point)) {
            mask_ptr[i] = 0;
        }
    }

    /// 2. we compute the derivatives in col / row direction
    ///    border points and points with invalid neighbours get special treatment
    /// the x direction
    cv::Mat xs(height, width, CV_32FC3, cv::Scalar());
    cv::Mat ys(height, width, CV_32FC3, cv::Scalar());

    cv::Vec3f *xs_ptr = xs.ptr<cv::Vec3f>();
    cv::Vec3f *ys_ptr = ys.ptr<cv::Vec3f>();
    uchar *normals_mask_ptr = normals_mask.ptr<uchar>();

    const std::size_t last_row = height - 1;
    const std::size_t last_col = width - 1;
    for(std::size_t i = 1 ; i < last_row; ++i) {
        const std::size_t row_begin = i * width;
        const std::size_t row_end = row_begin + last_col;
        for(std::size_t j = 1 ; j < row_end ; ++j) {
            const std::size_t pos = row_begin + j;
            if(mask_ptr[pos]) {
                /// d/dx
                const uchar x0 = mask_ptr[pos - 1];
                const uchar x1 = mask_ptr[pos + 1];
                if(x0 && x1) {
                    xs_ptr[pos] = points_ptr[pos - 1] - points_ptr[pos + 1];
                } else if(x0) {
                    xs_ptr[pos] = points_ptr[pos - 1] - points_ptr[pos];
                } else if(x1) {
                    xs_ptr[pos] = points_ptr[pos] - points_ptr[pos + 1];
                } else {
                    normals_mask_ptr[pos] = 0;
                }

                /// d/dy
                const uchar y0 = mask_ptr[pos - width];
                const uchar y1 = mask_ptr[pos + width];
                if(y0 && y1) {
                    ys_ptr[pos] = points_ptr[pos - width] - points_ptr[pos + width];
                } else if(y0) {
                    ys_ptr[pos] = points_ptr[pos - width] - points_ptr[pos];
                } else if(y1) {
                    ys_ptr[pos] = points_ptr[pos] - points_ptr[pos + width];
                } else {
                    normals_mask_ptr[0] = 0;
                }
            }
        }
    }

    for(std::size_t i = 0 ; i < height ; ++i) {
        const std::size_t row_begin = i * width;
        const std::size_t row_end = row_begin + last_col;
        if(mask_ptr[row_begin] && mask_ptr[row_begin + 1]) {
            xs_ptr[row_begin] = points_ptr[row_begin] - points_ptr[row_begin + 1];
        } else {
            normals_mask_ptr[row_begin] = 0;
        }
        if(mask_ptr[row_end] && mask_ptr[row_end - 1]) {
            xs_ptr[row_end] = points_ptr[row_end - 1] - points_ptr[row_end];
        } else {
            normals_mask_ptr[row_end] = 0;
        }
    }

    for(std::size_t j = 0 ; j < width ; ++j) {
        if(mask_ptr[j] && mask_ptr[j + width]) {
            ys_ptr[j] = points_ptr[j] - points_ptr[j + width];
        } else {
            normals_mask_ptr[j] = 0;
        }
        if(mask_ptr[last_row + j] && mask_ptr[last_row + j - width]) {
            ys_ptr[last_row + j] = points_ptr[last_row + j - width] - points_ptr[last_row + j];
        } else {
            normals_mask_ptr[last_row + j] = 0;
        }
    }

    /// 3. we apply some smoothing here
    /// we have to blur by hand ... 3x3 box scheme should be sufficient
    /// out of convienence, border entries are left as they are
    cv::Mat xs_smoothed(height, width, CV_32FC3, cv::Scalar());
    cv::Mat ys_smoothed(height, width, CV_32FC3, cv::Scalar());
    for(std::size_t i = 1 ; i < last_row ; ++i) {
        for(std::size_t j = 1 ; j < last_col ; ++j) {
            const cv::Rect roi(j-1,i-1, 3, 3);
            const cv::Mat kernel(normals_mask, roi);
            const double sum_kernel = cv::sum(kernel)[0];
            if(sum_kernel >= 2) {
                const double delta = 1.0 / sum_kernel;
                cv::filter2D(cv::Mat(xs, roi), cv::Mat(xs_smoothed, roi), CV_32F, kernel, cv::Point(-1,-1), delta);
                cv::filter2D(cv::Mat(ys, roi), cv::Mat(ys_smoothed, roi), CV_32F, kernel, cv::Point(-1,-1), delta);
            }
        }
    }

    /// 4. if a normal output is connected, we publish some debug information
    if(msg::isConnected(output_normals_)) {
        NormalsMessage::Ptr normals_msg(new NormalsMessage(cloud->header.frame_id, cloud->header.stamp));
        normals_msg->value.reset(new pcl::PointCloud<pcl::Normal>(width, height));
        for(std::size_t i = 0 ; i < height ; ++i) {
            for(std::size_t j = 0 ; j < width ; ++j) {
                const std::size_t pos = i * width + j;
                if(normals_mask_ptr[pos]) {
                    const cv::Vec3f &x = xs_ptr[pos];
                    const cv::Vec3f &y = ys_ptr[pos];
                    const cv::Vec3f c = x.ddot(y);
                    normals_msg->value->at(j, i) = {c[0], c[1], c[2]};
                }
            }
        }
        msg::publish(output_normals_, normals_msg);
}
}

