/// HEADER
#include "mask_refinement.h"

/// PROJECT
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_modifier.h>
#include <csapex/serialization/serialization.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/msg/io.h>
#include <csapex/view/utility/QtCvImageConverter.h>

/// SYSTEM
#include <QImage>

CSAPEX_REGISTER_CLASS(csapex::MaskRefinement, csapex::Node)

using namespace csapex;

MaskRefinement::MaskRefinement()
    : has_img_(false)
{
}


void MaskRefinement::setup(csapex::NodeModifier& node_modifier)
{
    in_mask_ = node_modifier.addInput<connection_types::CvMatMessage>("mask");
    in_img_ = node_modifier.addOptionalInput<connection_types::CvMatMessage>("image");

    out_ = node_modifier.addOutput<connection_types::CvMatMessage>("refined mask");
}

void MaskRefinement::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("submit"), std::bind(&MaskRefinement::ok, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("drop"), std::bind(&MaskRefinement::drop, this));

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("brush/size", 1, 64, 4, 1), [this](csapex::param::Parameter*){
        update_brush();
    });
}

void MaskRefinement::ok()
{
    next_image();
}

void MaskRefinement::drop()
{
    result_.reset();
    done();
}

void MaskRefinement::setMask(const QImage &mask)
{
    node_modifier_->setNoError();

    QtCvImageConverter::Converter<QImage>::QImage2Mat(mask).copyTo(mask_);
    cv::cvtColor(mask_, result_->value, CV_BGR2GRAY);

    done();
}


void MaskRefinement::beginProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters)
{
    auto in = msg::getMessage<connection_types::CvMatMessage>(in_mask_);
    mask_ = in->value;

    result_ = std::make_shared<connection_types::CvMatMessage>(in->getEncoding(), in->stamp_micro_seconds);

    QImage qmask = QtCvImageConverter::Converter<QImage>::mat2QImage(mask_);

    QImage qmasked;
    has_img_ = msg::isConnected(in_img_);
    if(has_img_) {
        img_ = msg::getMessage<connection_types::CvMatMessage>(in_img_)->value;
        qmasked = QtCvImageConverter::Converter<QImage>::mat2QImage(img_);
    }

    input(qmask, qmasked);

    if(has_img_ && mask_.size != img_.size) {
        node_modifier_->setWarning("The mask has not the same size as the image size");
    }
}

void MaskRefinement::finishProcess(csapex::NodeModifier& node_modifier, Parameterizable &parameters)
{
    if(result_) {
        msg::publish(out_, result_);
    }
}
