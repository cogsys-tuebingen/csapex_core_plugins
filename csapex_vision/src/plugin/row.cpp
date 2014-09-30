/// HEADER
#include "row.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Row, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

Row::Row()
{
}

void Row::process()
{
    CvMatMessage::Ptr in = input_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr out(new CvMatMessage(in->getEncoding()));

    if(in->value.empty()) {
        throw std::runtime_error("Cannot extract row of empty matrix!");
    }

    param::RangeParameter::Ptr range =
            getParameter<param::RangeParameter>("row");

    int max_idy = in->value.rows - 1;
    if(range->max<int>() != max_idy) {
        range->setMax(max_idy);
    }

    std::cout << range->max<int>() << std::endl;

    int index = readParameter<int>("row");
    out->value = in->value.row(index);

    output_->publish(out);
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
}
