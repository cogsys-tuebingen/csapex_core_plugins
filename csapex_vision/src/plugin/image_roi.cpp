/// HEADER
#include "image_roi.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_qt/QtCvImageConverter.h>
#include <csapex_scan_2d/scan_message.h>
#include <csapex_scan_2d/labeled_scan_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::ImageRoi, csapex::Node)

using namespace csapex;
using namespace connection_types;

ImageRoi::ImageRoi()
{
    addTag(Tag::get("General"));
    addTag(Tag::get("Vision"));

}

ImageRoi::~ImageRoi()
{
}

QIcon ImageRoi::getIcon() const
{
    return QIcon(":/picture.png");
}

void ImageRoi::setupParameters()
{
    addParameter(param::ParameterFactory::declareTrigger("submit", param::ParameterDescription("Continue with the current labeling")),
                                                         boost::bind(&ImageRoi::submit, this));

    addParameter(param::ParameterFactory::declareRange("roi",
                                                       param::ParameterDescription("The label to be assigned to the selected points"),
                                                       0, 9, 0, 1));
}

void ImageRoi::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("Image");
    output_ = modifier_->addOutput<RoiMessage>("Roi");
}

void ImageRoi::submit()
{
    submit_request();
}

void ImageRoi::process()
{
    InteractiveNode::process();

    result_.reset();

    CvMatMessage::Ptr in = input_->getMessage<CvMatMessage>();
    display_request(&in->value);

    waitForView();

    output_->publish(result_);
}


void ImageRoi::setResult(connection_types::LabeledScanMessage::Ptr result)
{
    result_ = result;
    done();
}
