/// HEADER
#include "label_rois.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/color_parameter.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <cslibs_vision/utils/cluster_boundaries.hpp>

#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/cv_mat_message.h>

#include <QColor>

CSAPEX_REGISTER_CLASS(csapex::LabelROIs, csapex::Node)

using namespace csapex;
using namespace connection_types;
using namespace csapex;

LabelROIs::LabelROIs()
{
}

LabelROIs::~LabelROIs()
{
}

void LabelROIs::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareTrigger("submit"), std::bind(&LabelROIs::submit, this));

    parameters.addParameter(csapex::param::factory::declareTrigger("drop"), std::bind(&LabelROIs::drop, this));

    parameters.addParameter(csapex::param::factory::declareTrigger("clear"), std::bind(&LabelROIs::clear, this));

    parameters.addParameter(csapex::param::factory::declareRange("class id", csapex::param::ParameterDescription("The class id to be used!"), 0, 255, 0, 1), std::bind(&LabelROIs::setClass, this));

    parameters.addParameter(csapex::param::factory::declareColorParameter("class color", 0, 255, 0), std::bind(&LabelROIs::setColor, this));
    setColor();
}

void LabelROIs::setup(NodeModifier& node_modifier)
{
    in_image_ = node_modifier.addInput<CvMatMessage>("Image");
    in_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("Rois");
    out_rois_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("Labeled Rois");
}

void LabelROIs::setActiveClassColor(const int r, const int g, const int b)
{
    csapex::param::ParameterPtr ptr = getParameter("class color");
    param::ColorParameter* col = (param::ColorParameter*)ptr.get();
    std::vector<int> rgb = { r, g, b };
    col->set(rgb);
}

void LabelROIs::submit()
{
    submit_request();
}

void LabelROIs::drop()
{
    drop_request();
}

void LabelROIs::clear()
{
    clear_request();
}

void LabelROIs::setColor()
{
    const std::vector<int>& cc = readParameter<std::vector<int>>("class color");
    set_color(cc[0], cc[1], cc[2]);
}

void LabelROIs::setClass()
{
    int c = readParameter<int>("class id");
    set_class(c);
}

void LabelROIs::display()
{
    if (image_.empty())
        return;

    QImage qimg = QtCvImageConverter::Converter::mat2QImage(image_);

    display_request(qimg);
}

void LabelROIs::beginProcess()
{
    CvMatMessage::ConstPtr in_img = msg::getMessage<CvMatMessage>(in_image_);
    std::shared_ptr<std::vector<RoiMessage> const> in_rois = msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);
    if (in_img->value.empty())
        return;

    /// COPY INPUT DATA
    rois_.clear();
    for (const RoiMessage& roi : *in_rois) {
        rois_.push_back(roi);
    }

    image_ = in_img->value.clone();

    /// RENDER CLUSTER BOARDERS
    setColor();
    setClass();
    display();
}

void LabelROIs::finishProcess()
{
    std::shared_ptr<std::vector<RoiMessage>> out_rois(new std::vector<RoiMessage>);
    out_rois->assign(rois_.begin(), rois_.end());
    msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, out_rois);
}
