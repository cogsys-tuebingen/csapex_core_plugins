/// HEADER
#include "col.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>
#include <utils_param/range_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Col, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

Col::Col() :
    request_center_(false)
{
}

void Col::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(input_);
    CvMatMessage::Ptr out(new CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));

    if(in->value.empty()) {
        throw std::runtime_error("Cannot extract row of empty matrix!");
    }

    param::RangeParameter::Ptr range =
            getParameter<param::RangeParameter>("col");

    int max_idx = in->value.cols - 1;
    if(range->max<int>() != max_idx) {
        range->setMax(max_idx);
    }

    if(request_center_) {
        int center = max_idx / 2;
        range->set(center);
        request_center_ = false;
    }

    int index = readParameter<int>("col");
    out->value = in->value.col(index);

    msg::publish(output_, out);
}

void Col::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("Matrix");
    output_ = modifier_->addOutput<CvMatMessage>("Col");
}

void Col::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("col",
                                                       param::ParameterDescription("Col to extract."),
                                                       0, 1, 0, 1));
    addParameter(param::ParameterFactory::declareTrigger("center"),
                 std::bind(&Col::requestCenter, this));

}

void Col::requestCenter()
{
    request_center_ = true;
}
