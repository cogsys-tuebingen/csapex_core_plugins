#include "ray_angles.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex_ros/time_stamp_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::RayAngles, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

RayAngles::RayAngles()
{
}

void RayAngles::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");

    output_general_     = node_modifier.addOutput<CvMatMessage>("Angles General");
    output_component_a_ = node_modifier.addOutput<CvMatMessage>("Angle 1. Component");
    output_component_b_ = node_modifier.addOutput<CvMatMessage>("Angle 2. Component");
}

void RayAngles::setupParameters(csapex::Parameterizable &parameters)
{
    std::map<std::string, int> reference_axis = {
        {"x", (int) X},
        {"y", (int) Y},
        {"z", (int) Z}
    };

    addParameter(csapex::param::ParameterFactory::declareParameterSet(
                     "reference axis",
                     csapex::param::ParameterDescription("Choose the reference axis."),
                     reference_axis,
                     (int) X));
}

void RayAngles::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));

    boost::apply_visitor (PointCloudMessage::Dispatch<RayAngles>(this, msg), msg->value);
}

namespace impl {
template <class PointT>
inline void processCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                         const Eigen::Vector3d &ref_axis,
                         cv::Mat &angles_general,
                         cv::Mat &angles_component_a,
                         cv::Mat &angles_component_b)
{
    unsigned n = cloud->points.size();

    int cols = cloud->width;
    int rows = n / cols;
    angles_general     = cv::Mat(rows, cols, CV_32FC1, cv::Scalar::all(0.0));
    angles_component_a = cv::Mat(rows, cols, CV_32FC1, cv::Scalar::all(0.0));
    angles_component_b = cv::Mat(rows, cols, CV_32FC1, cv::Scalar::all(0.0));

    auto   pt = cloud->points.begin();
    float* angles_general_ptr     = (float*) angles_general.ptr<float>();
    float* angles_component_a_ptr = (float*) angles_component_a.ptr<float>();
    float* angles_component_b_ptr = (float*) angles_component_b.ptr<float>();

    for(unsigned idx = 0; idx < n; ++idx) {
        const PointT &p = *pt;
        Eigen::Vector3d  pg(p.x, p.y, p.z);
        Eigen::Vector3d  pa(p.x, p.y, p.z);
        Eigen::Vector3d  pb(p.x, p.y, p.z);

        if(ref_axis(0) == 1.0) {
            pa(2) = 0.0;
            pb(1) = 0.0;
        } else if (ref_axis(1) == 1.0){
            pa(2) = 0.0;
            pb(0) = 0.0;
        } else {
            pa(1) = 0.0;
            pb(0) = 0.0;
        }

        double normax = ref_axis.norm();
        double dotg   = ref_axis.dot(pg);
        double normg  = pg.norm();
        double dota   = ref_axis.dot(pa);
        double norma  = pa.norm();
        double dotb   = ref_axis.dot(pb);
        double normb  = pb.norm();

        if(normax != 0) {
            if(normg != 0)
                *angles_general_ptr = acos(dotg / (normax * normg));
            if(norma != 0)
                *angles_component_a_ptr = acos(dota / (normax * norma));
            if(normb != 0)
                *angles_component_b_ptr = acos(dotb / (normax * normb));
        }

        ++angles_general_ptr;
        ++angles_component_a_ptr;
        ++angles_component_b_ptr;
        ++pt;
    }

}


/// specialization for 2d
inline void processCloud(typename pcl::PointCloud<pcl::PointXY>::ConstPtr cloud,
                         const Eigen::Vector2d &ref_axis,
                         cv::Mat &angles_general)
{
    unsigned n = cloud->points.size();

    int cols = cloud->width;
    int rows = n / cols;
    angles_general     = cv::Mat(rows, cols, CV_32FC1, cv::Scalar::all(0.0));

    auto   pt = cloud->points.begin();
    float* angles_general_ptr     = (float*) angles_general.ptr<float>();

    for(unsigned idx = 0; idx < n; ++idx) {
        const pcl::PointXY& p = *pt;
        Eigen::Vector2d     pg(p.x, p.y);
        double dot     = ref_axis.dot(pg);
        double normax  = ref_axis.norm();
        double normg   = pg.norm();

        if(normax != 0 && normg != 0) {
            *angles_general_ptr = acos(dot / (normax * normg));
        }

        ++angles_general_ptr;
        ++pt;
    }
}

template<class PointT>
struct Processor {
    static void invokeProcess(RayAngles *instance,
                              typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        instance->doProcess3D<PointT>(cloud);
    }
};

template<>
struct Processor<pcl::PointXY> {
    static void invokeProcess(RayAngles *instance,
                              pcl::PointCloud<pcl::PointXY>::ConstPtr cloud)
    {
        instance->doProcess2D(cloud);
    }
};

template<>
struct Processor<pcl::PointNormal> {
    static void invokeProcess(RayAngles *instance,
                              pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud)
    {
        throw std::runtime_error("Unsupported type 'PointNormal'!");
    }
};


}

template <class PointT>
void RayAngles::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    CvMatMessage::Ptr general(new CvMatMessage(enc::mono, cloud->header.stamp));
    CvMatMessage::Ptr component_a(new CvMatMessage(enc::mono, cloud->header.stamp));
    CvMatMessage::Ptr component_b(new CvMatMessage(enc::mono, cloud->header.stamp));

    impl::Processor<PointT>::invokeProcess(this, cloud);

    general->value = angles_general_.clone();
    component_a->value = angles_component_a_.clone();
    component_b->value = angles_component_b_.clone();

    msg::publish(output_general_, general);
    msg::publish(output_component_a_, component_a);
    msg::publish(output_component_b_, component_b);
}






template <class PointT>
void RayAngles::doProcess3D(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    RefAxis ref = (RefAxis) readParameter<int>("reference axis");
    Eigen::Vector3d ref_axis;
    switch(ref) {
    case X:
        ref_axis(0) = 1.0;
        ref_axis(1) = 0.0;
        ref_axis(2) = 0.0;
        break;
    case Y:
        ref_axis(0) = 0.0;
        ref_axis(1) = 1.0;
        ref_axis(2) = 0.0;
        break;
    default: // Z
        ref_axis(0) = 0.0;
        ref_axis(1) = 0.0;
        ref_axis(2) = 1.0;
        break;
    }

    impl::processCloud<PointT>(cloud,
                               ref_axis,
                               angles_general_,
                               angles_component_a_,
                               angles_component_b_);
}

void RayAngles::doProcess2D(pcl::PointCloud<pcl::PointXY>::ConstPtr cloud)
{
    RefAxis ref = (RefAxis) readParameter<int>("reference axis");
    Eigen::Vector2d ref_axis;
    switch(ref) {
    case Y:
        ref_axis(0) = 0.0;
        ref_axis(1) = 1.0;
        break;
    default: // Z || X
        ref_axis(0) = 1.0;
        ref_axis(1) = 0.0;
        node_modifier_->setWarning("No 'z'-axis for 2D Pointclouds, treated as X!");
        break;
    }

    impl::processCloud(cloud,
                       ref_axis,
                       angles_general_);

    angles_component_a_ = angles_general_;
    angles_component_b_ = angles_general_;
}

