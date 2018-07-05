/// HEADER
#include <csapex_scan_2d/renderer.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <boost/mpl/vector.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace lib_laser_processing;


Renderer::Renderer()
    : w(0), h(0), scale(1.0)
{
    addParameter(csapex::param::ParameterFactory::declareRange("width", 100, 2000, 256, 1));
    addParameter(csapex::param::ParameterFactory::declareRange("height", 100, 2000, 256, 1));

    addParameter(csapex::param::ParameterFactory::declareRange("scale", 0.1, 50.0, 5.0, 0.1));
    addParameter(csapex::param::ParameterFactory::declareRange("radius", 1.0, 5.0, 1.0, 0.1));
    addParameter(csapex::param::ParameterFactory::declareRange("rotation", -M_PI, M_PI, 0.0, 0.01));

    addParameter(csapex::param::ParameterFactory::declareBool("drawRays", true));
    addParameter(csapex::param::ParameterFactory::declareBool("drawHits", true));

    addParameter(csapex::param::ParameterFactory::declareColorParameter("color/hit", 0xFF, 0xCC, 0x00));
    addParameter(csapex::param::ParameterFactory::declareColorParameter("color/marked", 0xCC, 0xFF, 0x00));
    addParameter(csapex::param::ParameterFactory::declareBool("random_mark_color", false));
    addParameter(csapex::param::ParameterFactory::declareColorParameter("color/ray", 0xFF, 0xEE, 0xDD));
    addParameter(csapex::param::ParameterFactory::declareColorParameter("color/bg", 0x00, 0x00, 0x00));
}

void Renderer::drawRays(const Scan& scan, cv::Mat& img, const cv::Point2f& origin, cv::Scalar color, double angle_offset, double scale, double radius)
{
    float angle = scan.angle_min + angle_offset;
    float angle_step = scan.angle_increment;
    for(std::vector<LaserBeam>::const_iterator it = scan.rays.begin(); it != scan.rays.end(); ++it) {
        const LaserBeam& range = *it;

        if(range.valid()) {
            cv::Point2f pt(range.posX(), range.posY());
            cv::line(img, origin, origin + pt * scale, color, radius, CV_AA);
        }
        angle += angle_step;
    }
}
void Renderer::drawHits(const Scan& scan, cv::Mat& img, const cv::Point2f& origin, cv::Scalar color, cv::Scalar, double angle_offset, double scale, double radius)
{
    float angle = scan.angle_min + angle_offset;
    float angle_step = scan.angle_increment;
    for(std::vector<LaserBeam>::const_iterator it = scan.rays.begin(); it != scan.rays.end(); ++it) {
        const LaserBeam& range = *it;

        if(range.valid()) {
            cv::Point2f pt(range.posX(), range.posY());
            cv::circle(img, origin + pt * scale, radius, color, CV_FILLED, CV_AA);
        }
        angle += angle_step;
    }
}
void Renderer::drawHits(const LabeledScan& scan, cv::Mat& img, const cv::Point2f& origin, cv::Scalar color, cv::Scalar marked, double angle_offset, double scale, double radius)
{
    float angle = scan.angle_min + angle_offset;
    float angle_step = scan.angle_increment;

    std::vector<LaserBeam>::const_iterator range_it = scan.rays.begin();
    std::vector<int>::const_iterator label_it = scan.labels.begin();

    for(; range_it != scan.rays.end(); ++range_it, ++label_it) {
        const LaserBeam& range = *range_it;

        if(range.range() > 1e-10 && range.valid()) {

            int label = *label_it;

            cv::Point2f pt(range.posX(), range.posY());

            if(mark_random_color && label != 0) {
                color::fromCount(label, marked[2], marked[1], marked[0]);
            }

            cv::circle(img, origin + pt * scale, radius, label != 0 ? marked : color, CV_FILLED, CV_AA);
        }

        angle += angle_step;
    }
}

cv::Point2f Renderer::getOrigin() const
{
    return cv::Point2f(w/2, h/2);
}

double Renderer::getScale() const
{
    return scale;
}
