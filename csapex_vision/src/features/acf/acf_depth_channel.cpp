#include "acf_depth_channel.hpp"

#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex_ml/features_message.h>

CSAPEX_REGISTER_CLASS(csapex::vision::ACFDepthChannel, csapex::Node)

using namespace csapex;
using namespace csapex::vision;
using namespace csapex::connection_types;

void ACFDepthChannel::setup(csapex::NodeModifier& node_modifier)
{
    in_image_      = node_modifier.addInput<CvMatMessage>("Depth Map");
    in_rois_       = node_modifier.addOptionalInput<GenericVectorMessage, RoiMessage>("ROIs");
    out_channels_  = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Channel Features");
    out_visualize_ = node_modifier.addOutput<CvMatMessage>("Visualize");
}

void ACFDepthChannel::setupParameters(csapex::Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("window/width",
                                                                  10, 1024, 64, 1),
                            std::bind(&ACFDepthChannel::updateWindow, this));
    parameters.addParameter(param::ParameterFactory::declareRange("window/height",
                                                                  10, 1024, 128, 1),
                            std::bind(&ACFDepthChannel::updateWindow, this));
    parameters.addParameter(param::ParameterFactory::declareBool("window/keep_ratio",
                                                                 false),
                            keep_ratio_);
    parameters.addParameter(param::ParameterFactory::declareBool("window/mirror",
                                                                 false),
                            mirror_);

    parameters.addParameter(param::ParameterFactory::declareRange("aggregate/block_size",
                                                                  1, 32, 4, 1),
                            block_size_);

    static const std::map<std::string, int> available_types = {
            { "binary", static_cast<int>(Type::BINARY) },
            { "ternary", static_cast<int>(Type::TERNARY) },
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("channel/type",
                                                                        available_types,
                                                                        static_cast<int>(Type::BINARY)),
                            reinterpret_cast<int&>(type_));

    static const std::map<std::string, int> available_methods = {
            { "median", static_cast<int>(Method::MEDIAN) },
            { "mean", static_cast<int>(Method::MEAN) },
            { "histogram", static_cast<int>(Method::HISTOGRAM) },
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("channel/method",
                                                                        available_methods,
                                                                        static_cast<int>(Method::MEDIAN)),
                            reinterpret_cast<int&>(method_));

    parameters.addParameter(param::ParameterFactory::declareRange("channel/threshold",
                                                                 0.0, 1000.0, 0.1, 0.01),
                           threshold_);

   parameters.addParameter(param::ParameterFactory::declareBool("channel/normalize",
                                                                false),
                           normalize_);
}

void ACFDepthChannel::updateWindow()
{
    int old_height = window_height_;

    window_width_  = readParameter<int>("window/width");
    window_height_ = readParameter<int>("window/height");
    if (window_ratio_ == 0.0)
        window_ratio_ = window_height_ / double(window_width_);

    if (keep_ratio_)
    {
        if (window_height_ != old_height)
        {
            window_width_ = window_height_ / window_ratio_;
            setParameter<int>("window/width", window_width_);
        } else
        {
            window_height_ = window_width_ * window_ratio_;
            setParameter<int>("window/height", window_height_);
        }
    }
}

std::vector<float> ACFDepthChannel::extractChannel(const cv::Mat& depth_map) const
{
    cv::Mat aggregated_depth_map;
    cv::resize(depth_map, aggregated_depth_map, cv::Size(depth_map.cols / block_size_, depth_map.rows / block_size_));
    const cv::Mat valid_pixel_mask = aggregated_depth_map > 0;

    double min_value;
    double max_value;
    cv::minMaxLoc(aggregated_depth_map, &min_value, &max_value);

    float center = 0.0f;
    switch (method_)
    {
        case Method::HISTOGRAM:
        {
            const static float BIN_SIZE = 0.25f;
            static const int channels[] = { 0 };
            const int bins[] = { int((max_value - min_value) / BIN_SIZE) };
            const float value_range[] = { float(min_value), float(max_value) + std::numeric_limits<float>::epsilon() };
            const float* ranges[] = { value_range };

            cv::Mat hist;
            cv::calcHist(&aggregated_depth_map, 1, channels, valid_pixel_mask, hist, 1, bins, ranges, true);

            cv::Point max;
            cv::minMaxLoc(hist, nullptr, nullptr, nullptr, &max);

            center = min_value + (max.y + 0.5f) * BIN_SIZE;
            break;
        }
        case Method::MEDIAN:
        {
            cv::Mat values = aggregated_depth_map.reshape(0, 1).clone();
            const int invalid_pixel_count = values.cols - cv::countNonZero(valid_pixel_mask);

            // invalid points are <0 and thus will be at the beginning of the (partial) sort, so we have to skip them
            const auto middle = invalid_pixel_count + (values.cols - invalid_pixel_count) / 2;
            std::nth_element(values.begin<float>(), values.begin<float>() + middle, values.end<float>());
            center = values.at<float>(0, middle);
            break;
        }
        case Method::MEAN:
            center = cv::mean(aggregated_depth_map, valid_pixel_mask)[0];
            break;
    }

    std::vector<float> feature;
    feature.reserve(aggregated_depth_map.rows * aggregated_depth_map.cols);

    switch (type_)
    {
        case Type::TERNARY:
            std::transform(aggregated_depth_map.begin<float>(), aggregated_depth_map.end<float>(),
                           std::back_inserter(feature),
                           [&](float value)
                           {
                               if (normalize_)
                               {
                                   const float delta = value - center;
                                   if (delta > threshold_)
                                       return delta / float(max_value - (center + threshold_));
                                   else if (delta < -threshold_)
                                       return delta / float(center - threshold_ - min_value);
                                   else
                                       return 0.f;
                               }
                               else
                               {
                                   if (value > center + threshold_)
                                       return 1.f;
                                   else if (value < center - threshold_)
                                       return -1.f;
                                   else
                                       return 0.f;
                               }
                           });
            break;
        case Type::BINARY:
            std::transform(aggregated_depth_map.begin<float>(), aggregated_depth_map.end<float>(),
                           std::back_inserter(feature),
                           [&](float value)
                           {
                               if (normalize_)
                               {
                                   const float delta = value - center;
                                   if (delta > threshold_)
                                       return delta / float(max_value - (center + threshold_));
                                   else if (delta < -threshold_)
                                       return -delta / float(center - threshold_ - min_value);
                                   else
                                       return 0.f;
                               }
                               else
                               {
                                   if (std::abs(value - center) > threshold_)
                                       return 1.f;
                                   else
                                       return 0.f;
                               }
                           });
            break;
    }

    return std::move(feature);
}

void ACFDepthChannel::process()
{
    CvMatMessage::ConstPtr in_image = msg::getMessage<CvMatMessage>(in_image_);
    const cv::Mat& image = in_image->value;

    std::shared_ptr<std::vector<RoiMessage> const> in_rois;
    if (msg::hasMessage(in_rois_))
        in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    if (image.channels() != 1 || image.type() != CV_32F)
        throw std::runtime_error("Only 1 channel float images (depth maps) are supported");


    CvMatMessage::Ptr out_visualize;
    if (msg::isConnected(out_visualize_))
    {
        out_visualize = std::make_shared<CvMatMessage>(enc::bgr, in_image->frame_id, in_image->stamp_micro_seconds);
        out_visualize->frame_id = in_image->frame_id;
        out_visualize->value = cv::Mat(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    }


    auto out_features = std::make_shared<std::vector<FeaturesMessage>>();

    const auto process_roi = [&](const Roi& roi)
    {
        const cv::Rect roi_region = roi.rect() & cv::Rect(0, 0, image.cols, image.rows);

        FeaturesMessage feature(in_image->stamp_micro_seconds);
        feature.classification = roi.classification();

        cv::Mat image_region;
        cv::resize(cv::Mat(image, roi_region), image_region, cv::Size(window_width_, window_height_));

        feature.value = extractChannel(image_region);
        out_features->push_back(feature);

        if (out_visualize)
        {
            const float scale_x = float(image_region.cols / block_size_) / roi_region.width;
            const float scale_y = float(image_region.rows / block_size_) / roi_region.height;

            for (int dy = 0; dy < roi_region.height; ++dy)
                for (int dx = 0; dx < roi_region.width; ++dx)
                {
                    const int ldx = dx * scale_x;
                    const int ldy = dy * scale_y;
                    const int step = window_width_ / block_size_;

                    const float value = feature.value[ldx + step * ldy];
                    cv::Vec3b& dst = out_visualize->value.at<cv::Vec3b>(roi_region.y + dy, roi_region.x + dx);

                    if (value == 0)
                         dst = cv::Vec3b(0, 255, 0);
                    else if (value < 0)
                        dst = cv::Vec3b(0, 0, 255) * std::abs(value);
                    else if (value > 0)
                        dst = cv::Vec3b(255, 0, 0) * std::abs(value);
                }
        }

        if (mirror_)
        {
            cv::flip(image_region, image_region, 1);

            feature.value = extractChannel(image_region);
            out_features->push_back(std::move(feature));
        }
    };

    if (in_rois)
    {
        for (const RoiMessage& roi : *in_rois)
            process_roi(roi.value);
    }
    else
        process_roi(csapex::Roi(0, 0, image.cols, image.rows));


    msg::publish<GenericVectorMessage, FeaturesMessage>(out_channels_, out_features);
    if (out_visualize)
        msg::publish(out_visualize_, out_visualize);
}
