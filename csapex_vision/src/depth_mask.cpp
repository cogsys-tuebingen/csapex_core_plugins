#include "depth_mask.hpp"

#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>

CSAPEX_REGISTER_CLASS(csapex::vision::DepthMask, csapex::Node)

using namespace csapex;
using namespace csapex::vision;
using namespace csapex::connection_types;

void DepthMask::setup(csapex::NodeModifier& node_modifier)
{
    in_image_ = node_modifier.addInput<CvMatMessage>("Image");
    in_rois_ = node_modifier.addOptionalInput<GenericVectorMessage, RoiMessage>("ROIs");
    out_mask_ = node_modifier.addOutput<CvMatMessage>("Mask");
}

void DepthMask::setupParameters(csapex::Parameterizable& parameters)
{
    static const std::map<std::string, int> available_methods = {
        { "median", static_cast<int>(Method::MEDIAN) },
        { "mean", static_cast<int>(Method::MEAN) },
        { "histogram", static_cast<int>(Method::HISTOGRAM) },
    };
    parameters.addParameter(param::factory::declareParameterSet("method", available_methods, static_cast<int>(Method::HISTOGRAM)), reinterpret_cast<int&>(method_));

    parameters.addParameter(param::factory::declareRange("threshold", 0.0, 1000.0, 0.1, 0.01), threshold_);

    parameters.addParameter(param::factory::declareBool("invert", false), invert_);

    parameters.addParameter(param::factory::declareRange("offset/x", -640, 640, 0, 1), offset_x_);
    parameters.addParameter(param::factory::declareRange("offset/y", -480, 480, 0, 1), offset_y_);
}

void DepthMask::process()
{
    CvMatMessage::ConstPtr in_image = msg::getMessage<CvMatMessage>(in_image_);
    const cv::Mat& image = in_image->value;

    std::shared_ptr<std::vector<RoiMessage> const> in_rois;
    if (msg::hasMessage(in_rois_))
        in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    if (image.channels() != 1 || image.type() != CV_32F)
        throw std::runtime_error("Only 1 channel float images (depth maps) are supported");

    CvMatMessage::Ptr out_mask = std::make_shared<CvMatMessage>(enc::mono, in_image->frame_id, in_image->stamp_micro_seconds);
    out_mask->value = cv::Mat(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

    if (in_rois) {
        for (const RoiMessage& roi : *in_rois) {
            cv::Rect region = roi.value.rect() & cv::Rect(0, 0, image.cols, image.rows);
            processRegion(image(region), out_mask->value(region));
        }
    } else
        processRegion(image, out_mask->value);

    if (invert_)
        out_mask->value = 255 - out_mask->value;

    msg::publish(out_mask_, out_mask);
}

void DepthMask::processRegion(const cv::Mat& image, cv::Mat mask)
{
    const cv::Mat valid_pixel_mask = image > 0;

    double min_value;
    double max_value;
    cv::minMaxLoc(image, &min_value, &max_value);

    float center = 0.0f;
    switch (method_) {
        case Method::HISTOGRAM: {
            const static float BIN_SIZE = 0.25f;
            static const int channels[] = { 0 };
            const int bins[] = { int((max_value - min_value) / BIN_SIZE) };
            const float value_range[] = { float(min_value), float(max_value) + std::numeric_limits<float>::epsilon() };
            const float* ranges[] = { value_range };

            cv::Mat hist;
            cv::calcHist(&image, 1, channels, valid_pixel_mask, hist, 1, bins, ranges, true);

            cv::Point max;
            cv::minMaxLoc(hist, nullptr, nullptr, nullptr, &max);

            center = min_value + (max.y + 0.5f) * BIN_SIZE;
            break;
        }
        case Method::MEDIAN: {
            cv::Mat values = image.reshape(0, 1).clone();
            const int invalid_pixel_count = values.cols - cv::countNonZero(valid_pixel_mask);

            // invalid points are <0 and thus will be at the beginning of the (partial)
            // sort, so we have to skip them
            const auto middle = invalid_pixel_count + (values.cols - invalid_pixel_count) / 2;
            std::nth_element(values.begin<float>(), values.begin<float>() + middle, values.end<float>());
            center = values.at<float>(0, middle);
            break;
        }
        case Method::MEAN:
            center = cv::mean(image, valid_pixel_mask)[0];
            break;
    }

    for (int y = offset_y_; y < image.rows; ++y)
        for (int x = offset_x_; x < image.cols; ++x) {
            auto& mv = mask.at<uint8_t>(y - offset_y_, x - offset_x_);
            auto value = image.at<float>(y, x);

            if (value > 0)
                mv = std::abs(value - center) < threshold_ ? 255 : 0;
        }
}
