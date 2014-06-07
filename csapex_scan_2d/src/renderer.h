#ifndef RENDERER_H
#define RENDERER_H

/// PROJECT
#include <csapex/model/parameterizable.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {

class Renderer : public Parameterizable
{
public:
    Renderer();

    template <typename ScanType>
    void render(ScanType& scan, cv::Mat& output)
    {
        int w = param<int>("width");
        int h = param<int>("height");

        const std::vector<int>& color = param<std::vector<int> >("color/bg");
        cv::Scalar bgColor(color[2], color[1], color[0]);

        cv::Mat(h, w, CV_8UC3, bgColor).copyTo(output);

        cv::Point2f origin(w/2, h/2);
        double scale = param<double>("scale") * 10.0;
        double angle = param<double>("rotation");

        if(param<bool>("drawRays")) {
            const std::vector<int>& color = param<std::vector<int> >("color/ray");
            cv::Scalar rayColor(color[2], color[1], color[0]);
            drawRays(scan, output, origin, rayColor, angle, scale);
        }
        if(param<bool>("drawHits")) {
            const std::vector<int>& color = param<std::vector<int> >("color/hit");
            const std::vector<int>& marked = param<std::vector<int> >("color/marked");
            cv::Scalar hitColor(color[2], color[1], color[0]);
            cv::Scalar markedColor(marked[2], marked[1], marked[0]);
            drawHits(scan, output, origin, hitColor, markedColor, angle, scale);
        }
    }

protected:
    void drawRays(const lib_laser_processing::Scan& scan, cv::Mat& img, const cv::Point2f& origin, cv::Scalar color, double angle_offset, double scale);

    void drawHits(const lib_laser_processing::Scan& scan, cv::Mat& img, const cv::Point2f& origin, cv::Scalar color, cv::Scalar, double angle_offset, double scale);
    void drawHits(const lib_laser_processing::LabeledScan& scan, cv::Mat& img, const cv::Point2f& origin, cv::Scalar color, cv::Scalar marked, double angle_offset, double scale);
};

}

#endif // RENDERER_H
