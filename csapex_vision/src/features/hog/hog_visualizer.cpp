/// HEADER
#include "hog_visualizer.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_ml/features_message.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>

/// SYSTEM
#include <opencv2/objdetect/objdetect.hpp>

CSAPEX_REGISTER_CLASS(csapex::HOGVisualizer, csapex::Node)
/// TODO : L2HysThreshold - derivAperture

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

HOGVisualizer::HOGVisualizer()
{
}

void HOGVisualizer::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::factory::declareBool("hog/signed_gradient", param::ParameterDescription("Un-/directed gradients."), false), signed_gradient_);

    parameters.addParameter(param::factory::declareRange("hog/gradient_bins", param::ParameterDescription("Amount of gradient bins."), 2, 18, 9, 1), bins_);

    parameters.addParameter(param::factory::declareRange("hog/cells_x", param::ParameterDescription("Cells in x direction."), 2, 16, 8, 1), cells_x_);

    parameters.addParameter(param::factory::declareRange("hog/cells_y", param::ParameterDescription("Cells in x direction."), 2, 16, 16, 1), cells_y_);

    parameters.addParameter(param::factory::declareRange("hog/block_size", param::ParameterDescription("Cell count in both dimension of a block."), 1, 4, 2, 1), block_size_);
    parameters.addParameter(param::factory::declareRange("hog/bock_stride", param::ParameterDescription("Overlap of each block in cells."), 1, 3, 1, 1), block_stride_);
    parameters.addParameter(param::factory::declareRange("visuaization/scale_hog_image", param::ParameterDescription("Scale for the visualized images."), 1.0, 10.0, 4.0, 0.1), scale_hog_image_);
    parameters.addParameter(param::factory::declareRange("visuaization/scale_block_images", param::ParameterDescription("Scale for the visualized images."), 1.0, 200.0, 40.0, 0.1),
                            scale_block_images_);

    parameters.addParameter(param::factory::declareRange("visuaization/gradient_scale", param::ParameterDescription("Scale for the visualized images."), 1, 200, 8, 1), scale_gradient_);
}

void HOGVisualizer::setup(NodeModifier& node_modifier)
{
    in_img_ = node_modifier.addInput<CvMatMessage>("image");
    in_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("rois");
    in_positive_svm_weights_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("positive svm_weights");
    in_negative_svm_weights_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("negative svm weights");
    in_descriptors_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("descriptors");
    out_hog_images_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage>("rendered");
    out_positive_block_visualizations_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage>("positive blocks");
    out_negative_block_visualizations_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage>("negaitve blocks");
}

void HOGVisualizer::process()
{
    CvMatMessage::ConstPtr in_img = msg::getMessage<CvMatMessage>(in_img_);
    std::shared_ptr<std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);
    std::shared_ptr<std::vector<FeaturesMessage> const> in_positive_svm_weights = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_positive_svm_weights_);
    std::shared_ptr<std::vector<FeaturesMessage> const> in_negative_svm_weights = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_negative_svm_weights_);
    std::shared_ptr<std::vector<FeaturesMessage> const> in_descriptors = msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_descriptors_);

    if (in_img->value.channels() != 1 && in_img->value.channels() != 3) {
        throw std::runtime_error("Only 1 or 3 channel matrices supported!");
    }

    std::shared_ptr<std::vector<CvMatMessage>> out_positive_blocks_visualization(new std::vector<CvMatMessage>);
    std::shared_ptr<std::vector<CvMatMessage>> out_negative_blocks_visualization(new std::vector<CvMatMessage>);
    std::shared_ptr<std::vector<CvMatMessage>> out_hog_images(new std::vector<CvMatMessage>);

    const std::size_t size = in_rois->size();
    const RoiMessage* in_rois_ptr = in_rois->data();
    const FeaturesMessage* in_positive_svm_weights_ptr = in_positive_svm_weights->data();
    const FeaturesMessage* in_negative_svm_weights_ptr = in_negative_svm_weights->data();

    auto emplace_blocks = [&in_img](cv::Mat& blocks, std::vector<CvMatMessage>& blocks_visulalizations, bool invert, const double scale) {
        cv::normalize(blocks, blocks, 0, 255.0, cv::NORM_MINMAX);
        cv::Mat block_img;
        blocks.convertTo(block_img, CV_8UC1);
        CvMatMessage block_msg(enc::mono, in_img->frame_id, in_img->stamp_micro_seconds);
        if (invert) {
            block_msg.value = 255 - block_img;
        } else {
            block_msg.value = block_img.clone();
        }

        if (scale != 1.0) {
            cv::resize(block_msg.value, block_msg.value, cv::Size(), scale, scale, cv::InterpolationFlags::INTER_NEAREST);
        }

        blocks_visulalizations.emplace_back(block_msg);
    };

    for (std::size_t r = 0; r < size; ++r) {
        cv::Rect roi = in_rois_ptr[r].rect();
        const cv::Mat roi_img = cv::Mat(in_img->value, roi);

        CvMatMessage out_hog_image(enc::bgr, in_img->frame_id, in_img->stamp_micro_seconds);
        renderHOGFeatures(roi_img, in_descriptors->at(r).value, out_hog_image.value);
        out_hog_images->emplace_back(out_hog_image);

        const std::vector<float>& positive_weights = in_positive_svm_weights_ptr[r].value;
        const std::vector<float>& negative_weights = in_negative_svm_weights_ptr[r].value;
        cv::Mat postive_blocks = cv::Mat(cells_y_ - block_stride_, cells_x_ - block_stride_, CV_32FC1, cv::Scalar());
        cv::Mat negative_blocks = cv::Mat(cells_y_ - block_stride_, cells_x_ - block_stride_, CV_32FC1, cv::Scalar());
        for (int i = 0; i < cells_y_ - block_stride_; ++i) {
            for (int j = 0; j < cells_x_ - block_stride_; ++j) {
                postive_blocks.at<float>(i, j) = positive_weights[j * (cells_y_ - block_stride_) + i];
                negative_blocks.at<float>(i, j) = negative_weights[j * (cells_y_ - block_stride_) + i];
            }
        }
        emplace_blocks(postive_blocks, *out_positive_blocks_visualization, false, scale_block_images_);
        emplace_blocks(negative_blocks, *out_negative_blocks_visualization, true, scale_block_images_);
    }
    msg::publish<GenericVectorMessage, CvMatMessage>(out_hog_images_, out_hog_images);
    msg::publish<GenericVectorMessage, CvMatMessage>(out_positive_block_visualizations_, out_positive_blocks_visualization);
    msg::publish<GenericVectorMessage, CvMatMessage>(out_negative_block_visualizations_, out_negative_blocks_visualization);
}

void HOGVisualizer::renderHOGFeatures(const cv::Mat& src, const std::vector<float>& descriptor, cv::Mat& dst)
{
    // HOG descriptor length = #Blocks * #CellsPerBlock * #BinsPerCell
    //                      = (64/8-1) * (128/8-1) * (2*2) * 9
    //                      = 7        * 15        *  4    * 9
    //                      = 3780

    if (src.channels() == 1) {
        cv::cvtColor(src, dst, cv::COLOR_GRAY2BGR);
    } else {
        dst = src.clone();
    }

    if (scale_hog_image_ != 1.0) {
        cv::resize(dst, dst, cv::Size(), scale_hog_image_, scale_hog_image_, cv::INTER_LINEAR);
    }

    int cell_size_x = dst.cols / cells_x_;
    int cell_size_y = dst.rows / cells_y_;

    const int gradient_bins = bins_ * (signed_gradient_ ? 2.0 : 1.0);
    const float rad_range_bin = M_PI / static_cast<float>(gradient_bins);

    // prepare data structure: 9 orientation / gradient strenghts for each cell
    std::vector<cv::Mat> magnitudes;
    for (int i = 0; i < gradient_bins; ++i)
        magnitudes.emplace_back(cv::Mat(cells_y_, cells_x_, CV_32FC1, cv::Scalar()));
    cv::Mat cell_update_count = cv::Mat(cells_y_, cells_x_, CV_32SC1, cv::Scalar());

    // since there is a new block on each cell (overlapping blocks!) but the last
    // one
    const int blocks_in_x_dir = cells_x_ - block_stride_;
    const int blocks_in_y_dir = cells_y_ - block_stride_;

    // compute gradient strengths per cell
    int descriptor_idx = 0;

    for (int bx = 0; bx < blocks_in_x_dir; ++bx) {
        for (int by = 0; by < blocks_in_y_dir; ++by) {
            const static int dy[4] = { 0, 1, 0, 1 };
            const static int dx[4] = { 0, 0, 1, 1 };
            for (int i = 0; i < 4; ++i) {
                int cy = by + dy[i];
                int cx = bx + dx[i];
                for (int b = 0; b < gradient_bins; ++b) {
                    magnitudes[b].at<float>(cy, cx) += descriptor.at(descriptor_idx);
                    ++descriptor_idx;
                }
                ++cell_update_count.at<int>(cy, cx);
            }
        }
    }

    // compute average gradient strengths
    float magnitude_max = std::numeric_limits<float>::lowest();
    for (int cy = 0; cy < cells_y_; ++cy) {
        for (int cx = 0; cx < cells_x_; ++cx) {
            const float update_count = static_cast<float>(cell_update_count.at<int>(cy, cx));
            for (int b = 0; b < gradient_bins; ++b) {
                magnitudes[b].at<float>(cy, cx) /= static_cast<float>(update_count);
                magnitude_max = std::max(magnitudes[b].at<float>(cy, cx), magnitude_max);
            }
        }
    }

    // draw cells
    for (int cy = 0; cy < cells_y_; ++cy) {
        for (int cx = 0; cx < cells_x_; ++cx) {
            int draw_x = cx * cell_size_x;
            int draw_y = cy * cell_size_y;

            int mx = draw_x + cell_size_x / 2;
            int my = draw_y + cell_size_y / 2;

            // draw in each cell all 9 gradient strengths
            for (int b = 0; b < gradient_bins; ++b) {
                const float current_gradient_magnitude = magnitudes[b].at<float>(cy, cx);

                // no line to draw?
                if (current_gradient_magnitude == 0.f)
                    continue;

                const float rad = b * rad_range_bin + rad_range_bin / 2;

                const float dx = std::cos(rad);
                const float dy = std::sin(rad);

                // compute line coordinates
                float x1 = mx - dx * current_gradient_magnitude * scale_gradient_;
                float y1 = my - dy * current_gradient_magnitude * scale_gradient_;
                float x2 = mx + dx * current_gradient_magnitude * scale_gradient_;
                float y2 = my + dy * current_gradient_magnitude * scale_gradient_;

                // draw gradient dstalization
                cv::line(dst, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255 * current_gradient_magnitude / magnitude_max), 1, cv::LINE_AA);
            }
        }
    }
}
