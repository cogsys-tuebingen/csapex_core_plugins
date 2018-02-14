/// HEADER
#include "image_roi.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/range_parameter.h>

#include <csapex_opencv/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::ImageRoi, csapex::Node)

using namespace csapex;
using namespace connection_types;

ImageRoi::ImageRoi() :
    last_mat_size_(-1, -1)
{
}

ImageRoi::~ImageRoi()
{
}

void ImageRoi::setupParameters(Parameterizable& parameters)
{
    csapex::param::Parameter::Ptr method = csapex::param::ParameterFactory::declareBool("step",
                                                                        csapex::param::ParameterDescription("Step by step submission."),
                                                                        true);
    addParameter(method,
                 std::bind(&ImageRoi::submit, this));

    std::function<bool()> k_cond = (std::bind(&csapex::param::Parameter::as<bool>, method.get()));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareTrigger("submit"),
                            k_cond,
                            std::bind(&ImageRoi::submit, this));

    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareTrigger("drop"),
                            k_cond,
                            std::bind(&ImageRoi::drop, this));

    addParameter(csapex::param::ParameterFactory::declareRange("roi width",
                                                       csapex::param::ParameterDescription("Set the width of the roi."),
                                                       0, 640, 640, 1));
    addParameter(csapex::param::ParameterFactory::declareRange("roi height",
                                                       csapex::param::ParameterDescription("Set the width of the roi."),
                                                       0, 480, 480, 1));

    addParameter(csapex::param::ParameterFactory::declareRange("class label",
                                                       csapex::param::ParameterDescription("Assign a class label to roi."),
                                                       -1, 255, -1, 1));
}

void ImageRoi::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("Image");
    output_ = node_modifier.addOutput<RoiMessage>("Roi");
}

void ImageRoi::submit()
{
    submit_request();
}

void ImageRoi::drop()
{
    drop_request();
}

void ImageRoi::beginProcess()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(input_);
    if(in->value.empty())
        return;

    int in_rows = in->value.rows;
    int in_cols = in->value.cols;

    if(in_rows != last_mat_size_.height ||
            in_cols != last_mat_size_.width) {
        param::RangeParameter::Ptr width =
                getParameter<param::RangeParameter>("roi width");
        width->setInterval<int>(1, in_cols);
        width->triggerChange();

        param::RangeParameter::Ptr height =
                getParameter<param::RangeParameter>("roi height");
        height->setInterval<int>(1, in_rows);
        height->triggerChange();

        last_mat_size_.height = in->value.rows;
        last_mat_size_.width  = in->value.cols;
    }

    QImage img = QtCvImageConverter::Converter::mat2QImage(in->value);

    display_request(img);

    bool wait = readParameter<bool>("step");
    if(wait) {
        return;
    } else {
        done();
    }

}

void ImageRoi::finishProcess()
{
    if(result_) {
        int class_label = readParameter<int>("class label");
        result_->value.setClassification(class_label);
        msg::publish(output_, result_);
    }
}

void ImageRoi::setResult(connection_types::RoiMessage::Ptr result)
{
    result_ = result;
    done();
}
