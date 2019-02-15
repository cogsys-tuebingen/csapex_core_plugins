
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_point_cloud/msg/point_cloud_message.h>
using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class CloudMahalanobisDistance : public Node
{
public:
    CloudMahalanobisDistance() :
        first_(true),
        hold_(false)
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_cloud_ = modifier.addInput<PointCloudMessage>("PointCloud");
        in_cloud2_ = modifier.addOptionalInput<PointCloudMessage>("PointCloud");
        out_ = modifier.addOutput<double>("Mahalanobis distance");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareTrigger("hold_refernce_cloud"),
                            [this](param::Parameter*)
        {
            hold_ = true;
        });
        params.addParameter(param::factory::declareTrigger("rest_refernce_cloud"),
                            [this](param::Parameter*)
        {
            hold_ = false;
        });

        params.addParameter(param::factory::declareBool("hold_first_ference_cloud",false),
                            hold_first_);
    }

    void process() override
    {
        has_cloud2_ = msg::hasMessage(in_cloud2_);
        if(has_cloud2_){
            PointCloudMessage::ConstPtr msg_opt = msg::getMessage<PointCloudMessage>(in_cloud2_);
            boost::apply_visitor(PointCloudMessage::Dispatch<CloudMahalanobisDistance>(this, msg_opt), msg_opt->value);
        }

        PointCloudMessage::ConstPtr msg = msg::getMessage<PointCloudMessage>(in_cloud_);

        boost::apply_visitor(PointCloudMessage::Dispatch<CloudMahalanobisDistance>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        double dist = 0;
        if(first_ || has_cloud2_){
            if(first_){
                std::size_t dim = getDim<PointT>(cloud);
                mean_ = Eigen::VectorXd::Zero(dim);
                covariance_ = Eigen::MatrixXd::Zero(dim, dim);
            }
            if(!hold_){
                getMeanAndCov<PointT>(cloud, mean_,covariance_);
            }
            hold_ = first_ && hold_first_;
            if(first_ && !has_cloud2_){
                msg::publish(out_, dist);
                first_ = false;
                return;
            }
            has_cloud2_ = false;
            return;
        }

        dist = getDistance<PointT>(cloud);
        dist /= cloud->points.size();
        if(!hold_){
            getMeanAndCov<PointT>(cloud, mean_,covariance_);
        }
        msg::publish(out_, dist);

    }

    template <class PointT>
    double getDistance(typename pcl::PointCloud<PointT>::ConstPtr cloud) const
    {
        std::size_t N = cloud->points.size();
        Eigen::MatrixXd S = covariance_.inverse();
        double d2 = 0;
        for (std::size_t i = 0; i < N; ++i) {
            const PointT& pt = cloud->points[i];
            Eigen::VectorXd p = getEigenVector<PointT>(pt);
            Eigen::MatrixXd diff = p - mean_;
            Eigen::Matrix<double, 1,1 > scalar =  (diff.transpose() * S * diff).eval();
            d2 += scalar(0);
        }
        return std::sqrt(d2);
    }

    template <class PointT>
    void getMeanAndCov(typename pcl::PointCloud<PointT>::ConstPtr cloud, Eigen::VectorXd& mean, Eigen::MatrixXd& cov) const
    {
        std::size_t N = cloud->points.size();
        mean = Eigen::VectorXd::Zero(3);
        cov = Eigen::MatrixXd::Zero(3,3);

        for (std::size_t i = 0; i < N; ++i) {
            const PointT& pt = cloud->points[i];
            Eigen::VectorXd p = getEigenVector<PointT>(pt);
            mean += p;
        }
        mean /= N;

        for (std::size_t i = 0; i < N; ++i) {
            PointT pt = cloud->points[i];
            Eigen::VectorXd p = getEigenVector<PointT>(pt);
            Eigen::VectorXd diff = p - mean;
            Eigen::MatrixXd cov_i = (diff * diff.transpose().eval()).eval();
            cov += cov_i;
        }
        cov /= N;
    }


    template <class PointT>
    Eigen::VectorXd getEigenVector(const PointT& pt) const
    {
        Eigen::Matrix<double,3, 1> p;
        p(0) = static_cast<double>(pt.x);
        p(1) = static_cast<double>(pt.y);
        p(2) = static_cast<double>(pt.z);
        return p;
    }

    Eigen::VectorXd getEigenVector(const pcl::PointNormal& pt) const
    {
        Eigen::Matrix<double, 6, 1> p;
        p(0) = static_cast<double>(pt.x);
        p(1) = static_cast<double>(pt.y);
        p(2) = static_cast<double>(pt.z);
        p(3) = static_cast<double>(pt.normal_x);
        p(4) = static_cast<double>(pt.normal_y);
        p(5) = static_cast<double>(pt.normal_z);
        return p;
    }

    Eigen::VectorXd getEigenVector(const pcl::PointXYZRGB& pt) const
    {
        Eigen::Matrix<double, 6, 1> p;
        p(0) = static_cast<double>(pt.x);
        p(1) = static_cast<double>(pt.y);
        p(2) = static_cast<double>(pt.z);
        p(3) = static_cast<double>(pt.r);
        p(4) = static_cast<double>(pt.g);
        p(5) = static_cast<double>(pt.b);
        return p;
    }

    Eigen::VectorXd getEigenVector(const pcl::PointXYZRGBA& pt) const
    {
        Eigen::Matrix<double, 6, 1> p;
        p(0) = static_cast<double>(pt.x);
        p(1) = static_cast<double>(pt.y);
        p(2) = static_cast<double>(pt.z);
        p(3) = static_cast<double>(pt.r);
        p(4) = static_cast<double>(pt.g);
        p(5) = static_cast<double>(pt.b);
        return p;
    }


    Eigen::VectorXd getEigenVector(const pcl::PointXYZHSV& pt) const
    {
        Eigen::Matrix<double, 6, 1> p;
        p(0) = static_cast<double>(pt.x);
        p(1) = static_cast<double>(pt.y);
        p(2) = static_cast<double>(pt.z);
        p(3) = static_cast<double>(pt.h);
        p(4) = static_cast<double>(pt.s);
        p(5) = static_cast<double>(pt.v);
        return p;
    }

    template <class PointT>
    std::size_t getDim(typename pcl::PointCloud<PointT>::ConstPtr) const
    {
        return 3;
    }

    std::size_t getDim(typename pcl::PointCloud<pcl::PointNormal>::ConstPtr ) const
    {
        return 6;
    }

    std::size_t getDim(typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr) const
    {
        return 6;
    }

    std::size_t getDim(typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr) const
    {
        return 6;
    }

    std::size_t getDim(typename pcl::PointCloud<pcl::PointXYZHSV>::ConstPtr) const
    {
        return 6;
    }

private:
    bool first_;
    bool has_cloud2_;
    bool hold_;
    bool hold_first_;
    Input* in_cloud_;
    Input* in_cloud2_;
    Output* out_;
    Eigen::VectorXd mean_;
    Eigen::MatrixXd covariance_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::CloudMahalanobisDistance, csapex::Node)

