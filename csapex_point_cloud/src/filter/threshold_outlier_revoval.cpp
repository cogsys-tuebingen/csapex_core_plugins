/// HEADER
#include "threshold_outlier_revoval.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/timestamp_message.h>
#include <csapex_opencv/cv_mat_message.h>

/// SYSTEM
#include <pcl/point_types.h>

CSAPEX_REGISTER_CLASS(csapex::ThresholdOutlierRemoval, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

namespace
{
template <typename Any>
class PointTraits
{
    typedef char Small;
    class Big
    {
        char dummy[2];
    };

    template <typename Class>
    static Small testx(__typeof__(&Class::x));
    template <typename Class>
    static Big testx(...);

    template <typename Class>
    static Small testy(__typeof__(&Class::y));
    template <typename Class>
    static Big testy(...);

    template <typename Class>
    static Small testz(__typeof__(&Class::z));
    template <typename Class>
    static Big testz(...);

public:
    enum
    {
        HasX = sizeof(testx<Any>(0)) == sizeof(Small)
    };
    enum
    {
        HasY = sizeof(testy<Any>(0)) == sizeof(Small)
    };
    enum
    {
        HasZ = sizeof(testz<Any>(0)) == sizeof(Small)
    };
    enum
    {
        HasXY = HasX && HasY && !HasZ
    };
    enum
    {
        HasXYZ = HasX && HasY && HasZ
    };
};

template <typename PointT>
inline static void add(const PointT& p1, PointT& p2, typename std::enable_if<PointTraits<PointT>::HasXY, PointT>::type* dummy = 0)
{
    p2.x += p1.x;
    p2.y += p1.y;
}

template <typename PointT>
inline static void add(const PointT& p1, PointT& p2, typename std::enable_if<PointTraits<PointT>::HasXYZ, PointT>::type* dummy = 0)
{
    p2.x += p1.x;
    p2.y += p1.y;
    p2.z += p1.z;
}

template <typename PointT, typename T>
inline static void divide(const T scalar, PointT& p1, typename std::enable_if<PointTraits<PointT>::HasXY, PointT>::type* dummy = 0)
{
    p1.x /= scalar;
    p1.y /= scalar;
}

template <typename PointT, typename T>
inline static void divide(const T scalar, PointT& p1, typename std::enable_if<PointTraits<PointT>::HasXYZ, PointT>::type* dummy = 0)
{
    p1.x /= scalar;
    p1.y /= scalar;
    p1.z /= scalar;
}

template <typename PointT, typename T>
inline static T distance(const PointT& p1, const PointT& p2, typename std::enable_if<PointTraits<PointT>::HasXY, PointT>::type* dummy = 0)
{
    T diffx = p1.x - p2.x;
    T diffy = p1.y - p2.y;
    return std::sqrt(diffx * diffx + diffy * diffy);
}

template <typename PointT, typename T>
inline static T distance(const PointT& p1, const PointT& p2, typename std::enable_if<PointTraits<PointT>::HasXYZ, PointT>::type* dummy = 0)
{
    T diffx = p1.x - p2.x;
    T diffy = p1.y - p2.y;
    T diffz = p1.z - p2.z;
    return std::sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
}

template <typename PointT>
struct ThresholdNoiseFilter
{
    inline static void interpolate(const typename pcl::PointCloud<PointT>::ConstPtr& src, const cv::Mat& thresholds, const uchar threshold, const double max_distance,
                                   typename pcl::PointCloud<PointT>::Ptr& dst)
    {
        static int xs[] = { -1, -1, -1, 0, 1, 1, 1, 0 };
        static int ys[] = { -1, 0, 1, 1, 1, 0, -1, -1 };

        dst.reset(new pcl::PointCloud<PointT>(*src));

        const PointT* src_ptr = src->points.data();
        PointT* dst_ptr = dst->points.data();
        const uchar* th_ptr = thresholds.ptr<uchar>();

        int cols = src->width;
        int rows = src->height;

        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                int pos = y * cols + x;
                if (th_ptr[pos] > threshold) {
                    double normalizer = 0;
                    PointT inter_value;
                    for (int i = 0; i < 8; ++i) {
                        int posx = x + xs[i];
                        int posy = y + ys[i];
                        if (posx > -1 && posx < cols && posy > -1 && posy < rows) {
                            int npos = posy * cols + posx;
                            if (th_ptr[npos] <= threshold && distance<PointT, double>(src_ptr[npos], src_ptr[pos]) <= max_distance) {
                                add(src_ptr[npos], inter_value);
                                normalizer += 1.0;
                            }
                        }
                    }
                    divide(normalizer, inter_value);
                    dst_ptr[pos] = inter_value;
                }
            }
        }
    }

    inline static void filter(const typename pcl::PointCloud<PointT>::ConstPtr& src, const cv::Mat& thresholds, const uchar threshold, typename pcl::PointCloud<PointT>::Ptr& dst)
    {
        dst.reset(new pcl::PointCloud<PointT>(*src));

        PointT* dst_ptr = dst->points.data();
        const uchar* th_ptr = thresholds.ptr<uchar>();

        int cols = src->width;
        int rows = src->height;

        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                int pos = y * cols + x;
                if (th_ptr[pos] > threshold) {
                    dst_ptr[pos] = PointT();
                }
            }
        }
    }
};
}  // namespace

ThresholdOutlierRemoval::ThresholdOutlierRemoval()
{
}

void ThresholdOutlierRemoval::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("threshold", 0, 255, 255, 1));
    parameters.addParameter(csapex::param::factory::declareRange("max. distance", 0.0, 10.0, 0.25, 0.01));
    parameters.addParameter(csapex::param::factory::declareBool("interpolate", false));
}

void ThresholdOutlierRemoval::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    thresholds_ = node_modifier.addInput<CvMatMessage>("Thresholds");
    output_ = node_modifier.addOutput<PointCloudMessage>("Filtered Pointcloud");
}

void ThresholdOutlierRemoval::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_));
    boost::apply_visitor(PointCloudMessage::Dispatch<ThresholdOutlierRemoval>(this, msg), msg->value);
}

template <class PointT>
void ThresholdOutlierRemoval::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    CvMatMessage::ConstPtr thresholds = msg::getMessage<CvMatMessage>(thresholds_);
    uchar threshold = readParameter<int>("threshold");
    double max_dist = readParameter<double>("max. distance");
    bool interpolate = readParameter<bool>("interpolate");

    if (thresholds->value.rows != (int)cloud->height)
        throw std::runtime_error("Height of pointcloud and threshold matrix not matching!");
    if (thresholds->value.cols != (int)cloud->width)
        throw std::runtime_error("Width of pointcloud and threshold matrix not matching!");
    if (!thresholds->hasChannels(1, CV_8U))
        throw std::runtime_error("Threshold matrix type must be 'mono'!");

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered;

    if (interpolate)
        ThresholdNoiseFilter<PointT>::interpolate(cloud, thresholds->value, threshold, max_dist, cloud_filtered);
    else
        ThresholdNoiseFilter<PointT>::filter(cloud, thresholds->value, threshold, cloud_filtered);

    PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
    out->value = cloud_filtered;
    msg::publish(output_, out);
}
