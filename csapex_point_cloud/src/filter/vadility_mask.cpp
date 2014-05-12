/// HEADER
#include "vadility_mask.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

CSAPEX_REGISTER_CLASS(csapex::VadilityMask, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

VadilityMask::VadilityMask()
{
}

void VadilityMask::process()
{
    PointCloudMessage::Ptr msg(input_->getMessage<PointCloudMessage>());

    boost::apply_visitor (PointCloudMessage::Dispatch<VadilityMask>(this), msg->value);
}

void VadilityMask::setup()
{
    input_  = addInput<PointCloudMessage>("PointCloud");
    output_ = addOutput<CvMatMessage>("Mask");
}

namespace implementation {
template<class PointT>
struct Impl {
    static void convert(const typename pcl::PointCloud<PointT>::Ptr cloud, cv::Mat &mask)
    {
        int height = cloud->height;
        int width  = cloud->width;
        mask   = cv::Mat(height, width, CV_8UC1, 255);

        for(int i = 0 ; i < height ; ++i) {
            for(int j = 0 ; j < width ; ++j) {
                PointT pos = cloud->at(i * width + j);
                if(pos.x == 0.f && pos.y == 0.f && pos.z == 0.f) {
                    mask.at<uchar>(i,j) = 0;
                }
            }
        }
    }
};
template <>
struct Impl<pcl::PointXY> {
    static void convert(const typename pcl::PointCloud<pcl::PointXY>::Ptr cloud, cv::Mat &mask)
    {
        std::runtime_error("Conversion is not supported for pcl::PointXY!");
    }
};

template<>
struct Impl<pcl::PointXYZI> {
    static void convert(const typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, cv::Mat &mask)
    {
        int height = cloud->height;
        int width  = cloud->width;
        mask   = cv::Mat(height, width, CV_8UC1, 255);

        for(int i = 0 ; i < height ; ++i) {
            for(int j = 0 ; j < width ; ++j) {
                pcl::PointXYZI pos = cloud->at(i * width + j);
                if(pos.x == 0.f && pos.y == 0.f && pos.z == 0.f) {
                    mask.at<uchar>(i,j) = 0;
                }
            }
        }
    }
};

}


template <class PointT>
void VadilityMask::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    #warning "FIX ENCODING"
    CvMatMessage::Ptr out(new CvMatMessage(enc::mono));
    implementation::Impl<PointT>::convert(cloud, out->value);
    output_->publish(out);
}
