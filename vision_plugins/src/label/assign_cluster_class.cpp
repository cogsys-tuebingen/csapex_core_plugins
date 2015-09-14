/// HEADER
#include "assign_cluster_class.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/view/utility/QtCvImageConverter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/color_parameter.h>
#include <csapex/param/range_parameter.h>
#include <utils_vision/utils/cluster_boundaries.hpp>

#include <csapex_vision/cv_mat_message.h>
#include <csapex_core_plugins/vector_message.h>

#include <QColor>

CSAPEX_REGISTER_CLASS(vision_plugins::AssignClusterClass, csapex::Node)

using namespace csapex;
using namespace connection_types;
using namespace vision_plugins;

AssignClusterClass::AssignClusterClass()
{
}

AssignClusterClass::~AssignClusterClass()
{
}

void AssignClusterClass::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareTrigger("submit"),
                            std::bind(&AssignClusterClass::submit, this));

    parameters.addParameter(param::ParameterFactory::declareTrigger("drop"),
                            std::bind(&AssignClusterClass::drop, this));

    parameters.addParameter(param::ParameterFactory::declareTrigger("clear"),
                            std::bind(&AssignClusterClass::clear, this));

    parameters.addParameter(param::ParameterFactory::declareRange("class id",
                                                                  param::ParameterDescription("The class id to be used!"),
                                                                  0, 255, 0, 1),
                                                                  std::bind(&AssignClusterClass::setClass, this));

    parameters.addParameter(param::ParameterFactory::declareColorParameter("class color", 0, 255, 0),
                            std::bind(&AssignClusterClass::setColor, this));
    parameters.addParameter(param::ParameterFactory::declareColorParameter("border color", 0, 0, 0),
                            std::bind(&AssignClusterClass::display, this));

    setColor();
}

void AssignClusterClass::setup(NodeModifier& node_modifier)
{
    in_image_    = node_modifier.addInput<CvMatMessage>("Image");
    in_clusters_ = node_modifier.addInput<CvMatMessage>("Clusters");
    out_labels_  = node_modifier.addOutput<GenericVectorMessage, int>("labels");
}

void AssignClusterClass::setActiveClassColor(const int r, const int g, const int b)
{
    param::ParameterPtr ptr = getParameter("class color");
    param::ColorParameter *col = (param::ColorParameter*) ptr.get();
    std::vector<int> rgb = boost::assign::list_of(r)(g)(b);
    col->set(rgb);
}

void AssignClusterClass::submit()
{
    submit_request();
}

void AssignClusterClass::drop()
{
    drop_request();
}

void AssignClusterClass::clear()
{
    clear_request();
}

void AssignClusterClass::setColor()
{
    const std::vector<int>& cc = readParameter<std::vector<int> >("class color");
    set_color(cc[0], cc[1], cc[2]);
}

void AssignClusterClass::setClass()
{
    int c = readParameter<int>("class id");
    set_class(c);
}

void AssignClusterClass::display()
{
    if(image_.empty())
        return;

    /// GET PARAMETERS
    const std::vector<int>& bc = readParameter<std::vector<int> >("border color");
    cv::Scalar border_color(bc[2], bc[1], bc[0]);

    image_.setTo(border_color, mask_);

    QImage qimg = QtCvImageConverter::Converter<QImage>::mat2QImage(image_);

    display_request(qimg, clusters_);
}


void AssignClusterClass::beginProcess()
{
    CvMatMessage::ConstPtr in_img = msg::getMessage<CvMatMessage>(in_image_);
    CvMatMessage::ConstPtr in_clu = msg::getMessage<CvMatMessage>(in_clusters_);
    if(in_img->value.empty())
        return;
    if(in_img->value.rows != in_clu->value.rows)
        throw std::runtime_error("Matrix row counts are not matchin!");
    if(in_img->value.cols != in_clu->value.cols)
        throw std::runtime_error("Matrix column counts are not matchin!");
    if(in_clu->value.type() != CV_32SC1)
        throw std::runtime_error("Cluster matrix has to be one channel integer!");


    /// RENDER CLUSTER BOARDERS
    image_ = in_img->value.clone();
    clusters_ = in_clu->value;
    utils_vision::getClusterBoundaryMask<int32_t>(clusters_, mask_);

    setColor();
    setClass();
    display();
}

void AssignClusterClass::finishProcess()
{
    if(result_) {
       msg::publish<GenericVectorMessage, int>(out_labels_, result_);
    }
}

void AssignClusterClass::setResult(std::vector<int> &result)
{
    if(result.empty())
        result_.reset();

    result_ = std::shared_ptr<std::vector<int>>(new std::vector<int>(result));
    done();
}
