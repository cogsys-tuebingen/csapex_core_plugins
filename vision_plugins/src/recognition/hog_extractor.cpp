/// HEADER
#include "hog_extractor.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <utils_cv/color_functions.hpp>
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_ml/features_message.h>

/// SYSTEM
#include <opencv2/objdetect/objdetect.hpp>
#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(vision_plugins::HOGExtractor, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

HOGExtractor::HOGExtractor()
{
}

void HOGExtractor::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("gaussian sigma",
                                                       param::ParameterDescription("Standard deviation for Gaussian blur."),
                                                       0.0, 10.0, 0.0, 0.1));

    addParameter(param::ParameterFactory::declareBool("gamma correction",
                                                      param::ParameterDescription("Enable the gamma correction."),
                                                      true));

    addParameter(param::ParameterFactory::declareRange("orientation bins",
                                                       param::ParameterDescription("Amount of histogram bins per block."),
                                                       2, 18, 9, 1));

    addParameter(param::ParameterFactory::declareRange("cell size",
                                                       param::ParameterDescription("Set the size of a cell"),
                                                       8, 80, 8, 1));


    param::Parameter::Ptr cells_per_block =
            param::ParameterFactory::declareRange("cells per block",
                                                  param::ParameterDescription("Set the amount of cells in each direction of block."),
                                                  2, 8, 2, 1);

    addParameter(cells_per_block, boost::bind(&HOGExtractor::updateOverlap, this));

    boost::function<bool()> k_cond = (boost::bind(&param::Parameter::as<int>, cells_per_block.get()) > 2);

    param::ParameterPtr o = param::ParameterFactory::declareRange(
                "overlap",
                param::ParameterDescription("Block overlap given in cells."),
                1, 7, 1, 1);
    overlap_ = std::dynamic_pointer_cast<param::RangeParameter>(o);

    addConditionalParameter(overlap_, k_cond);
}

void HOGExtractor::setup()
{
    in_img_     = modifier_->addInput<CvMatMessage>("image");
    in_rois_    = modifier_->addOptionalInput<GenericVectorMessage, RoiMessage>("rois");
    out_        = modifier_->addOutput<GenericVectorMessage, FeaturesMessage>("features");
}

void HOGExtractor::process()
{
    CvMatMessage::ConstPtr  in = in_img_->getMessage<CvMatMessage>();
    std::shared_ptr<std::vector<FeaturesMessage> > out(new std::vector<FeaturesMessage>);

    if(!in->hasChannels(1, CV_8U))
        throw std::runtime_error("Image must be one channel grayscale!");

    const cv::Mat &value = in->value;

    double gauss            = readParameter<double>("gaussian sigma");
    bool   gamma            = readParameter<bool>("gamma correction");
    int    bins             = readParameter<int>("orientation bins");
    int    cells_per_block  = readParameter<int>("cells per block");
    int    cell_size        = readParameter<int>("cell size");
    int    overlap          = readParameter<int>("overlap");

    int    block_size_px = cells_per_block * cell_size;
    int    overlap_px    = overlap * cell_size;

    cv::HOGDescriptor d(cv::Size(value.cols, value.rows),
                        cv::Size(block_size_px, block_size_px),
                        cv::Size(overlap_px, overlap_px),
                        cv::Size(cell_size, cell_size),
                        bins,
                        0.2,
                        gauss == 0.0 ? -1 : gauss,
                        gamma);

    if(!in_rois_->hasMessage()) {
        if(value.rows < block_size_px)
            throw std::runtime_error("Image must have at least block height in px.");
        if(value.cols < block_size_px)
            throw std::runtime_error("Image must have at least block width in px.");
        if(value.rows % cell_size != 0)
            throw std::runtime_error("Images have height as multiple of cell size!");
        if(value.cols % cell_size != 0)
            throw std::runtime_error("Images have width as multiple of cell size!");

        FeaturesMessage feature_msg;

        if(!value.empty())
            d.compute(value, feature_msg.value);

        feature_msg.classification = 0;

        out->push_back(feature_msg);

    } else {
        std::shared_ptr<std::vector<RoiMessage> const> in_rois =
                in_rois_->getMessage<GenericVectorMessage, RoiMessage>();

        for(std::vector<RoiMessage>::const_iterator
            it = in_rois->begin() ;
            it != in_rois->end() ;
            ++it) {

            FeaturesMessage feature_msg;
            cv::Rect const &rect = it->value.rect();

            d.winSize = cv::Size(rect.width, rect.height);

            cv::Mat roi_mat = cv::Mat(value, rect);

            if(!roi_mat.empty())
                d.compute(roi_mat, feature_msg.value);

            feature_msg.classification = it->value.classification();

            out->push_back(feature_msg);
        }
    }

    //    /// BLOCK STEPS X * BLOCK STEPS Y * BINS * CELLS (WITHIN BLOCK)

    out_->publish<GenericVectorMessage, FeaturesMessage>(out);
}

void HOGExtractor::updateOverlap()
{
    if(overlap_->isEnabled()) {
        int cells_per_block = readParameter<int>("cells per block");
        overlap_->setMax(cells_per_block - 1);
        overlap_->set(1);
    }
}
