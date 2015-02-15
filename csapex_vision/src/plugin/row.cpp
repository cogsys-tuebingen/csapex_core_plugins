/// HEADER
#include "row.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/range_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Row, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

Row::Row() :
    request_center_(false)
{
}

void Row::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(input_);
    CvMatMessage::Ptr out(new CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));

    if(in->value.empty()) {
        throw std::runtime_error("Cannot extract row of empty matrix!");
    }

    param::RangeParameter::Ptr range =
            getParameter<param::RangeParameter>("row");

    int max_idy = in->value.rows - 1;
    if(range->max<int>() != max_idy) {
        range->setMax(max_idy);

    }

    if(request_center_) {
        int center = max_idy / 2;
        range->set(center);
        request_center_ = false;
    }

    int index = readParameter<int>("row");
    out->value = in->value.row(index);

    msg::publish(output_, out);
}

void Row::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("Matrix");
    output_ = modifier_->addOutput<CvMatMessage>("Row");
}

void Row::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("row",
                                                       param::ParameterDescription("Row to extract."),
                                                       0, 1, 0, 1));
    addParameter(param::ParameterFactory::declareTrigger("center"),
                 std::bind(&Row::requestCenter, this));

}

void Row::requestCenter()
{
    request_center_ = true;
}
