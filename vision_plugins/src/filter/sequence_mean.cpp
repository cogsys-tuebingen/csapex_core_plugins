/// HEADER
#include "sequence_mean.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/normalization.hpp>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <boost/assign.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::SequenceMean, csapex::Node)

SequenceMean::SequenceMean() :
    sequence_size_(1)
{
}

void SequenceMean::process()
{
    CvMatMessage::ConstPtr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->stamp_micro_seconds));
    cv::Mat tmp = in->value.clone();

    /// CLEAR ACCUMULATED SEQUENCE, IF type_ or size_ CHANGE
    if(!check(tmp) || acc_.size() == 0) {
        acc_.clear();
        type_       = tmp.type();
        size_       = cv::Size(tmp.cols, tmp.rows);
        channels_   = tmp.channels();
    }
    /// POP IF MAXIMUM QUEUE SIZE IS REACHED
    while(acc_.size() >= sequence_size_) {
        acc_.pop_front();
    }
    /// COMPUTE
    if((tmp.type() & 7) != CV_32F)
        tmp.convertTo(tmp, CV_32FC(channels_));

    acc_.push_back(tmp);
    cv::Mat buff(size_.height, size_.width, CV_32FC(channels_), cv::Scalar::all(0));
    for(std::deque<cv::Mat>::iterator it = acc_.begin() ; it != acc_.end() ; ++it) {
        if(buff.type() != it->type())
            aerr << "autsch" << std::endl;
        cv::add(buff, *it, buff);
    }
    buff.convertTo(out->value,  type_, 1.0 / (double) acc_.size());
    output_->publish(out);
}

void SequenceMean::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("original");
    output_ = modifier_->addOutput<CvMatMessage>("filtered");
    update();
}

void SequenceMean::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("acc", 1, 100, (int) sequence_size_, 1),
                 std::bind(&SequenceMean::update, this));
}

void SequenceMean::update()
{
    sequence_size_ = (unsigned int) readParameter<int>("acc");
}

bool SequenceMean::check(const cv::Mat &mat)
{
    return
    mat.type()     == type_         &&
    mat.rows       == size_.height  &&
    mat.cols       == size_.width   &&
    mat.channels() == channels_;
}
