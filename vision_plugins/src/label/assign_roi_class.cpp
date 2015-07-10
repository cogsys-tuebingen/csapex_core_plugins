/// HEADER
#include "assign_roi_class.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <utils_qt/QtCvImageConverter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <utils_param/color_parameter.h>
#include <utils_param/range_parameter.h>
#include <utils_vision/utils/cluster_boundaries.hpp>

#include <csapex_vision/cv_mat_message.h>
#include <csapex_core_plugins/vector_message.h>


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
    parameters.addParameter(param::ParameterFactory::declareTrigger("submit"),
                            std::bind(&AssignROIClass::submit, this));

    parameters.addParameter(param::ParameterFactory::declareTrigger("drop"),
                            std::bind(&AssignROIClass::drop, this));

    parameters.addParameter(param::ParameterFactory::declareTrigger("clear"),
                            std::bind(&AssignROIClass::clear, this));

    parameters.addParameter(param::ParameterFactory::declareRange("class id",
                                                                  param::ParameterDescription("The class id to be used!"),
                                                                  0, 255, 0, 1),
                                                                  std::bind(&AssignROIClass::setClass, this));

    parameters.addParameter(param::ParameterFactory::declareColorParameter("class color", 0, 255, 0),
                            std::bind(&AssignROIClass::setColor, this));
    setColor();
}

void AssignROIClass::setup(NodeModifier& node_modifier)
{
    in_image_    = node_modifier.addInput<CvMatMessage>("Image");
    in_rois_ = node_modifier.addInput<VectorMessage, RoiMessage>("Rois");
    out_rois_  = node_modifier.addOutput<VectorMessage, RoiMessage>("Labeled Rois");
}

void AssignROIClass::setActiveClassColor(const int r, const int g, const int b)
{
    param::ParameterPtr ptr = getParameter("class color");
    param::ColorParameter *col = (param::ColorParameter*) ptr.get();
    std::vector<int> rgb = boost::assign::list_of(r)(g)(b);
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
    VectorMessage::ConstPtr in_rois = msg::getMessage<VectorMessage>(in_rois_);

    if(in_img->value.empty())
        return;

    /// COPY INPUT DATA
    rois_.clear();
    for(std::vector<ConnectionType::ConstPtr>::const_iterator it = in_rois->value.begin(); it != in_rois->value.end(); ++it) {
        RoiMessage::ConstPtr roi = std::dynamic_pointer_cast<RoiMessage const>(*it);
        RoiMessage::Ptr roi_msg(new RoiMessage);
        roi_msg->value = roi->value;
        rois_.push_back(roi_msg);
    }
    image_    = in_img->value.clone();

    /// RENDER CLUSTER BOARDERS
    setColor();
    setClass();
    display();
}

void AssignROIClass::finishProcess()
{
    VectorMessage::Ptr out_rois(VectorMessage::make<RoiMessage>());
    out_rois->value.assign(rois_.begin(), rois_.end());
    msg::publish(out_rois_, out_rois);
}
