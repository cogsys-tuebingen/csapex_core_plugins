/// HEADER
#include "hog_visualizer.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/roi_message.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_ml/features_message.h>

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
    parameters.addParameter(param::ParameterFactory::declareBool("hog/signed_gradient",
                                                                 param::ParameterDescription("Un-/directed gradients."),
                                                                 false),
                            signed_gradient_);

    parameters.addParameter(param::ParameterFactory::declareRange("hog/gradient_bins",
                                                                  param::ParameterDescription("Amount of gradient bins."),
                                                                  2, 18, 9, 1),
                            bins_);

    parameters.addParameter(param::ParameterFactory::declareRange("hog/cells_x",
                                                                  param::ParameterDescription("Cells in x direction."),
                                                                  2, 16, 8, 1),
                            cells_x_);

    parameters.addParameter(param::ParameterFactory::declareRange("hog/cells_y",
                                                                  param::ParameterDescription("Cells in x direction."),
                                                                  2, 16, 16, 1),
                            cells_y_);

    parameters.addParameter(param::ParameterFactory::declareRange("hog/cell_size",
                                                                  param::ParameterDescription("Size of the cells."),
                                                                  4, 16, 8, 1),
                            cell_size_);
    parameters.addParameter(param::ParameterFactory::declareRange("hog/block_size",
                                                                  param::ParameterDescription("Cell count in both dimension of a block."),
                                                                  1, 4, 2, 1),
                            block_size_);
    parameters.addParameter(param::ParameterFactory::declareRange("hog/bock_stride",
                                                                  param::ParameterDescription("Overlap of each block in cells."),
                                                                  1, 3, 1, 1),
                            block_stride_);
}

void HOGVisualizer::setup(NodeModifier& node_modifier)
{
    in_img_         = node_modifier.addInput<CvMatMessage>("image");
    in_rois_        = node_modifier.addInput<GenericVectorMessage, RoiMessage>("rois");
    in_positive_svm_weights_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("positive svm_weights");
    in_negative_svm_weights_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("negative svm weights");
    in_descriptors_ = node_modifier.addInput<GenericVectorMessage, FeaturesMessage>("descriptors");
    out_svm_weights_img_      = node_modifier.addOutput<CvMatMessage>("rendered");
    out_positive_block_visualizations_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage>("positive blocks");
    out_negative_block_visualizations_ = node_modifier.addOutput<GenericVectorMessage, CvMatMessage>("negaitve blocks");

}

void HOGVisualizer::process()
{
    CvMatMessage::ConstPtr  in_img = msg::getMessage<CvMatMessage>(in_img_);
    std::shared_ptr<std::vector<RoiMessage> const> in_rois =
            msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);
    std::shared_ptr<std::vector<FeaturesMessage> const> in_positive_svm_weights =
            msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_positive_svm_weights_);
    std::shared_ptr<std::vector<FeaturesMessage> const> in_negative_svm_weights =
            msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_negative_svm_weights_);
    std::shared_ptr<std::vector<FeaturesMessage> const> in_descriptors =
            msg::getMessage<GenericVectorMessage, FeaturesMessage>(in_descriptors_);

    if(in_img->value.channels() != 1 && in_img->value.channels() != 3) {
        throw std::runtime_error("Only 1 or 3 channel matrices supported!");
    }

    CvMatMessage::Ptr out_svm_weights_image(new CvMatMessage(enc::bgr, in_img->frame_id, in_img->stamp_micro_seconds));
    out_svm_weights_image->value = in_img->value.clone();

    std::shared_ptr<std::vector<CvMatMessage>> out_positive_blocks_visualization(new std::vector<CvMatMessage>);
    std::shared_ptr<std::vector<CvMatMessage>> out_negative_blocks_visualization(new std::vector<CvMatMessage>);

    const std::size_t      size = in_rois->size();
    const RoiMessage      *in_rois_ptr = in_rois->data();
    const FeaturesMessage *in_positive_svm_weights_ptr = in_positive_svm_weights->data();
    const FeaturesMessage *in_negative_svm_weights_ptr = in_negative_svm_weights->data();

    auto emplace_blocks = [&in_img](cv::Mat &blocks, std::vector<CvMatMessage> &blocks_visulalizations, bool invert)
    {
        cv::normalize(blocks, blocks, 0, 255.0, cv::NORM_MINMAX);
        cv::Mat block_img;
        blocks.convertTo(block_img, CV_8UC1);
        CvMatMessage block_msg(enc::mono, in_img->frame_id, in_img->stamp_micro_seconds);
        if(invert) {
            block_msg.value = 255 - block_img;
        } else {
            block_msg.value = block_img.clone();
        }
        blocks_visulalizations.emplace_back(block_msg);
    };

    for(std::size_t r = 0 ; r < size ; ++r) {
        cv::Rect roi = in_rois_ptr[r].rect();

        const std::vector<float> &positive_weights = in_positive_svm_weights_ptr[r].value;
        const std::vector<float> &negative_weights = in_negative_svm_weights_ptr[r].value;
        cv::Mat postive_blocks = cv::Mat(cells_y_ - block_stride_, cells_x_ - block_stride_, CV_32FC1, cv::Scalar());
        cv::Mat negative_blocks = cv::Mat(cells_y_ - block_stride_, cells_x_ - block_stride_, CV_32FC1, cv::Scalar());
        for(int i = 0 ; i < cells_y_ - block_stride_; ++i) {
            for(int j = 0 ; j < cells_x_ - block_stride_; ++j) {
                postive_blocks.at<float>(i,j) = positive_weights[j * (cells_y_ - block_stride_) + i];
                negative_blocks.at<float>(i,j) = negative_weights[j * (cells_y_ - block_stride_) + i];
            }
        }
        emplace_blocks(postive_blocks, *out_positive_blocks_visualization, false);
        emplace_blocks(negative_blocks, *out_negative_blocks_visualization, true);
    }
    msg::publish(out_svm_weights_img_, out_svm_weights_image);
    msg::publish<GenericVectorMessage, CvMatMessage>(out_positive_block_visualizations_, out_positive_blocks_visualization);
    msg::publish<GenericVectorMessage, CvMatMessage>(out_negative_block_visualizations_, out_negative_blocks_visualization);


}
