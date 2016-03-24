/// HEADER
#include "hog_extractor.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_ml/features_message.h>

/// SYSTEM
#include <opencv2/objdetect/objdetect.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::HOGExtractor, csapex::Node)
/// TODO : L2HysThreshold - derivAperture

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

HOGExtractor::HOGExtractor()
{
}

void HOGExtractor::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("hog/sigma",
                                                       param::ParameterDescription("Standard deviation for Gaussian blur."),
                                                       0.0, 10.0, 0.0, 0.1),
                 hog_.winSigma);

    parameters.addParameter(param::ParameterFactory::declareBool("hog/gamma_correction",
                                                      param::ParameterDescription("Enable the gamma correction."),
                                                      true),
                 hog_.gammaCorrection);
    parameters.addParameter(param::ParameterFactory::declareBool("hog/signed_gradient",
                                                      param::ParameterDescription("Un-/directed gradients."),
                                                      hog_.signedGradient),
                 hog_.signedGradient);

    addParameter(param::ParameterFactory::declareRange("hog/gradient_bins",
                                                      param::ParameterDescription("Amount of gradient bins."),
                                                      2, 18, hog_.nbins, 1),
                 hog_.nbins);

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
    parameters.addParameter(param::ParameterFactory::declareRange("hog/block size",
                                                                  param::ParameterDescription("Cell count in both dimension of a block."),
                                                                  1, 4, 2, 1),
                            block_size_);
    parameters.addParameter(param::ParameterFactory::declareRange("hog/bock_stride",
                                                                  param::ParameterDescription("Overlap of each block in cells."),
                                                                  1, 3, 1, 1),
                            block_stride_);

}

void HOGExtractor::setup(NodeModifier& node_modifier)
{
    in_img_     = node_modifier.addInput<CvMatMessage>("image");
    in_rois_    = node_modifier.addOptionalInput<VectorMessage, RoiMessage>("rois");
    out_        = node_modifier.addOutput<GenericVectorMessage, FeaturesMessage>("features");
}

void HOGExtractor::process()
{
    CvMatMessage::ConstPtr  in = msg::getMessage<CvMatMessage>(in_img_);
    VectorMessage::Ptr      out(VectorMessage::make<FeaturesMessage>());

    hog_.cellSize.width  = cell_size_;
    hog_.cellSize.height = cell_size_;
    hog_.winSize.height  = cell_size_ * cells_y_;
    hog_.winSize.width   = cell_size_ * cells_x_;
    hog_.blockSize       = hog_.cellSize * block_size_;
    hog_.blockStride     = hog_.cellSize * block_stride_;

    msg::publish(out_, out);
}
