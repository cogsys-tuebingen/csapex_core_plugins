/// HEADER
#include "reshape.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/range_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Reshape, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

Reshape::Reshape() :
    in_size_ (0),
    in_rows_ (0),
    in_cols_ (0),
    out_rows_(0),
    out_cols_(0),
    reset_(false)
{
}

void Reshape::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(input_);
    CvMatMessage::Ptr out(new CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));
    if((unsigned int) in->value.rows != in_rows_ ||
       (unsigned int) in->value.cols != in_cols_) {
        in_rows_ = in->value.rows;
        in_cols_ = in->value.cols;
        in_size_ = in_rows_ * in_cols_;
        param::RangeParameter::Ptr range_rows = getParameter<param::RangeParameter>("rows");
        param::RangeParameter::Ptr range_cols = getParameter<param::RangeParameter>("cols");
        range_rows->setInterval<int>(1, in_size_);
        range_cols->setInterval<int>(1, in_size_);
        range_rows->set<int>(in_rows_);
        range_cols->set<int>(in_cols_);
        out_rows_ = in_rows_;
        out_cols_ = in_cols_;
    }
    unsigned int out_rows = readParameter<int>("rows");
    unsigned int out_cols = readParameter<int>("cols");
    if(out_rows != out_rows_ && in_size_ % out_rows == 0) {
        param::RangeParameter::Ptr range_cols = getParameter<param::RangeParameter>("cols");

        unsigned int adapted_cols = in_size_ / out_rows;
        out_rows_ = out_rows;
        out_cols_ = adapted_cols;
        range_cols->set<int>(adapted_cols);
    } else if (out_cols != out_cols_ && in_size_ % out_cols == 0) {
        param::RangeParameter::Ptr range_rows = getParameter<param::RangeParameter>("rows");

        unsigned int adapted_rows = in_size_ / out_cols;
        out_cols_ = out_cols;
        out_rows_ = adapted_rows;
        range_rows->set<int>(adapted_rows);
    } else if (reset_) {
        param::RangeParameter::Ptr range_rows = getParameter<param::RangeParameter>("rows");
        param::RangeParameter::Ptr range_cols = getParameter<param::RangeParameter>("cols");
        range_rows->set<int>(in_rows_);
        range_cols->set<int>(in_cols_);
        reset_ = false;
    }
    out->value = in->value.clone();
    out->value = out->value.reshape(in->value.channels(), out_rows_);

    msg::publish(output_, out);
}

void Reshape::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("Matrix");
    output_ = node_modifier.addOutput<CvMatMessage>("Reshaped");
}

void Reshape::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::ParameterFactory::declareRange("rows",
                                                       csapex::param::ParameterDescription("New row count."),
                                                       1, 1, 1, 1));
    addParameter(csapex::param::ParameterFactory::declareRange("cols",
                                                       csapex::param::ParameterDescription("New col count."),
                                                       1, 1, 1, 1));
    addParameter(csapex::param::ParameterFactory::declareTrigger("reset",
                                                         csapex::param::ParameterDescription("Reset to default size.")),
                                                         std::bind(&Reshape::reset, this));
}

void Reshape::reset()
{
    reset_ = true;
}
