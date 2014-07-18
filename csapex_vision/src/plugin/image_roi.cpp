/// HEADER
#include "image_roi.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_qt/QtCvImageConverter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::ImageRoi, csapex::Node)

using namespace csapex;
using namespace connection_types;

ImageRoi::ImageRoi() :
    last_mat_size_(-1, -1)
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
    param::Parameter::Ptr method = param::ParameterFactory::declareBool("step",
                                                                        param::ParameterDescription("Step by step submission."),
                                                                        true);
    addParameter(method,
                 boost::bind(&ImageRoi::submit, this));

    boost::function<bool()> k_cond = (boost::bind(&param::Parameter::as<bool>, method.get()));
    addConditionalParameter(param::ParameterFactory::declareTrigger("submit"),
                            k_cond,
                            boost::bind(&ImageRoi::submit, this));

    addParameter(param::ParameterFactory::declareRange("roi width", param::ParameterDescription("Set the width of the roi."),
                                                       0, 640, 640, 1));
    addParameter(param::ParameterFactory::declareRange("roi height", param::ParameterDescription("Set the width of the roi."),
                                                       0, 480, 480, 1));
}

void ImageRoi::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("Image", false, true);
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
    if(in->value.empty())
        return;

    int in_rows = in->value.rows;
    int in_cols = in->value.cols;

    if(in_rows != last_mat_size_.height ||
            in_cols != last_mat_size_.width) {
        param::RangeParameter::Ptr width =
                getParameter<param::RangeParameter>("roi width");
        width->setInterval<int>(1, in_cols);
        width->set(in_cols);
        width->triggerChange();

        param::RangeParameter::Ptr height =
                getParameter<param::RangeParameter>("roi height");
        height->setInterval<int>(1, in_rows);
        height->set(in_rows);
        height->triggerChange();

        last_mat_size_.height = in->value.rows;
        last_mat_size_.width  = in->value.cols;
    }

    QSharedPointer<QImage> img =
            QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(in->value);

    display_request(img);

    bool wait = param<bool>("step");
    if(wait)
        waitForView();

    // output_->publish(result_);
}


void ImageRoi::setResult(connection_types::RoiMessage::Ptr result)
{
    result_ = result;
    done();
}
