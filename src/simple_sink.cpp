/// HEADER
#include "simple_sink.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/view/box.h>
#include <opencv2/opencv.hpp>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <QLabel>

CSAPEX_REGISTER_CLASS(csapex::SimpleSink, csapex::Node)


using namespace csapex;
using namespace connection_types;

SimpleSink::SimpleSink()
    : input_(NULL),output_(NULL), sunk(0)
{
}

void SimpleSink::fill(QBoxLayout *layout)
{
    if(input_ == NULL || output_ == NULL) {
        /// add input
        input_ = addInput<CvMatMessage>("Image");

        /// add output
        output_ = addOutput<CvMatMessage>("Image");

        /// debug output
        label = new QLabel;
        layout->addWidget(label);
    }
}

void SimpleSink::messageArrived(ConnectorIn *source)
{
    ++sunk;

    std::stringstream txt;
    txt << "sunk: " << sunk;
    label->setText(txt.str().c_str());

    CvMatMessage::Ptr m = source->getMessage<CvMatMessage>();

    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);
    for(uint i = 0 ; i < channels.size() ; i++) {
        cv::equalizeHist(channels[i], channels[i]);
    }
    cv::merge(channels, m->value);

    output_->publish(m);
}
