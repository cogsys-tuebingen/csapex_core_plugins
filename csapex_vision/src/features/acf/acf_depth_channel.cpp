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
    in_image_     = node_modifier.addInput<CvMatMessage>("Depth Map");
    in_rois_      = node_modifier.addOptionalInput<GenericVectorMessage, RoiMessage>("ROIs");
    out_channels_ = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("Channel Features");
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
    };
    parameters.addParameter(param::ParameterFactory::declareParameterSet("channel/method",
                                                                        available_methods,
                                                                        static_cast<int>(Method::MEDIAN)),
                            reinterpret_cast<int&>(method_));

    parameters.addParameter(param::ParameterFactory::declareRange("channel/threshold",
                                                                 0.0, 1000.0, 0.1, 0.01),
                           threshold_);
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

void ACFDepthChannel::extractChannel(const cv::Mat& depth, cv::Mat& channel)
{
    double center = 0.0;
    switch (method_)
    {
        case Method::MEDIAN:
        {
            cv::Mat continuous;
            cv::Mat values;

            if (depth.isContinuous())
                values = depth.reshape(0, 1).clone();
            else
            {
                continuous = depth.clone();
                values = continuous.reshape(0, 1);
            }

            const auto middle = values.cols / 2;
            std::nth_element(values.begin<float>(), values.begin<float>() + middle, values.end<float>());
            center = values.at<float>(0, middle);
            break;
        }
        case Method::MEAN:
            center = cv::mean(depth)[0];
            break;
    }

    channel.create(depth.rows, depth.cols, CV_8SC1);

    switch (type_)
    {
        case Type::TERNARY:
            std::transform(depth.begin<float>(), depth.end<float>(), channel.begin<int8_t>(),
                           [center, this](float value)
                           {
                               if (value > center + threshold_)
                                   return 1;
                               else if (value < center - threshold_)
                                   return -1;
                               else
                                   return 1;
                           });
            break;
        case Type::BINARY:
            std::transform(depth.begin<float>(), depth.end<float>(), channel.begin<int8_t>(),
                           [center, this](float value)
                           {
                               if (value > center + threshold_)
                                   return 1;
                               else
                                   return -1;
                           });
            break;
    }
}

void ACFDepthChannel::process()
{
    CvMatMessage::ConstPtr in_image = msg::getMessage<CvMatMessage>(in_image_);
    cv::Mat image = in_image->value;

    std::shared_ptr<std::vector<RoiMessage> const> in_rois;
    if (msg::hasMessage(in_rois_))
        in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);

    if (image.channels() != 1 || image.type() != CV_32F)
        throw std::runtime_error("Only 1 channel float depth images are supported");

    // 1) resize image to window
    // 2) aggregate values through resize (controlled by block_size)
    cv::resize(image, image, cv::Size(window_width_ / block_size_, window_height_ / block_size_));

    auto out_features = std::make_shared<std::vector<FeaturesMessage>>();
    auto process_roi = [&](const Roi& roi)
    {
        auto extract_feature = [&](const cv::Mat& region)
        {
            FeaturesMessage feature(in_image->stamp_micro_seconds);

            cv::Mat channel;
            extractChannel(region, channel);

            feature.classification = roi.classification();
            feature.value.reserve(channel.rows * channel.cols);
            channel.copyTo(feature.value);
            return feature;
        };

        cv::Mat image_region(image, roi.rect());
        out_features->push_back(extract_feature(image_region));

        if (mirror_)
        {
            cv::flip(image_region, image_region, 1);
            out_features->push_back(extract_feature(image_region));
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
}
