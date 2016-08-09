/// HEADER
#include "assign_roi_class.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/color_parameter.h>
#include <csapex/param/range_parameter.h>
#include <cslibs_vision/utils/cluster_boundaries.hpp>

#include <csapex_vision/cv_mat_message.h>
#include <csapex/msg/generic_vector_message.hpp>


#include <QColor>

CSAPEX_REGISTER_CLASS(vision_plugins::AssignROIClass, csapex::Node)

using namespace csapex;
using namespace connection_types;
using namespace vision_plugins;

AssignROIClass::AssignROIClass()
{
}

AssignROIClass::~AssignROIClass()
{
}

void AssignROIClass::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("submit"),
                            std::bind(&AssignROIClass::submit, this));

    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("drop"),
                            std::bind(&AssignROIClass::drop, this));

    parameters.addParameter(csapex::param::ParameterFactory::declareTrigger("clear"),
                            std::bind(&AssignROIClass::clear, this));

    parameters.addParameter(csapex::param::ParameterFactory::declareRange("class id",
                                                                  csapex::param::ParameterDescription("The class id to be used!"),
                                                                  0, 255, 0, 1),
                                                                  std::bind(&AssignROIClass::setClass, this));

    parameters.addParameter(csapex::param::ParameterFactory::declareColorParameter("class color", 0, 255, 0),
                            std::bind(&AssignROIClass::setColor, this));
    setColor();
}

void AssignROIClass::setup(NodeModifier& node_modifier)
{
    in_image_    = node_modifier.addInput<CvMatMessage>("Image");
    in_rois_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("Rois");
    out_rois_  = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("Labeled Rois");
}

void AssignROIClass::setActiveClassColor(const int r, const int g, const int b)
{
    csapex::param::ParameterPtr ptr = getParameter("class color");
    param::ColorParameter *col = (param::ColorParameter*) ptr.get();
    std::vector<int> rgb = {r, g, b};
    col->set(rgb);
}

void AssignROIClass::submit()
{
    submit_request();
}

void AssignROIClass::drop()
{
    drop_request();
}

void AssignROIClass::clear()
{
    clear_request();
}

void AssignROIClass::setColor()
{
    const std::vector<int>& cc = readParameter<std::vector<int> >("class color");
    set_color(cc[0], cc[1], cc[2]);
}

void AssignROIClass::setClass()
{
    int c = readParameter<int>("class id");
    set_class(c);
}

void AssignROIClass::display()
{
    if(image_.empty())
        return;

    QImage qimg = QtCvImageConverter::Converter<QImage>::mat2QImage(image_);

    display_request(qimg);
}


void AssignROIClass::beginProcess()
{
    CvMatMessage::ConstPtr  in_img  = msg::getMessage<CvMatMessage>(in_image_);
    std::shared_ptr<std::vector<RoiMessage> const> in_rois =
            msg::getMessage<GenericVectorMessage, RoiMessage>(in_rois_);
    if(in_img->value.empty())
        return;

    /// COPY INPUT DATA
    rois_.clear();
    for(const RoiMessage &roi : *in_rois) {
        rois_.push_back(roi);
    }

    image_    = in_img->value.clone();

    /// RENDER CLUSTER BOARDERS
    setColor();
    setClass();
    display();
}

void AssignROIClass::finishProcess()
{
    std::shared_ptr<std::vector<RoiMessage>> out_rois(new std::vector<RoiMessage>);
    out_rois->assign(rois_.begin(), rois_.end());
    msg::publish<GenericVectorMessage, RoiMessage>(out_rois_, out_rois);
}
