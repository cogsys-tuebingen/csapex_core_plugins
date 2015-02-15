/// HEADER
#include "grab_cut.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex/utility/timer.h>

/// SYSTEM
#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::GrabCut, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


GrabCut::GrabCut()
{
}

void GrabCut::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("iterations",
                                                       param::ParameterDescription("Number of iterations of GrabCut"),
                                                       1, 100, 1, 1));

    addParameter(param::ParameterFactory::declareRange("threshold",
                                                       param::ParameterDescription("Minimum value for a mask pixel to be used as non-zero"),
                                                       0, 255, 128, 1));

    std::map<std::string, int> init_value = boost::assign::map_list_of
            ("background", (int) cv::GC_BGD)
            ("foreground", (int) cv::GC_FGD)
            ("most probably background", (int) cv::GC_PR_BGD)
            ("most probably foreground", (int) cv::GC_PR_FGD);

    addParameter(param::ParameterFactory::declareParameterSet<int>("initial value",
                                                                   param::ParameterDescription("Initial mask value for unspecified pixels"),
                                                                   init_value, cv::GC_BGD));
}

void GrabCut::setup()
{
    in_  = modifier_->addInput<CvMatMessage>("Image");
    in_fg_  = modifier_->addInput<CvMatMessage>("certain foreground");
    in_bg_  = modifier_->addInput<CvMatMessage>("certain background");

    in_roi_  = modifier_->addOptionalInput<RoiMessage>("ROI");

    out_fg_ = modifier_->addOutput<CvMatMessage>("possible foreground");
    out_bg_ = modifier_->addOutput<CvMatMessage>("possible background");
}

void GrabCut::process()
{
    INTERLUDE("process");

    CvMatMessage::ConstPtr img_msg = msg::getMessage<CvMatMessage>(in_);
    CvMatMessage::ConstPtr fg_msg = msg::getMessage<CvMatMessage>(in_fg_);
    CvMatMessage::ConstPtr bg_msg = msg::getMessage<CvMatMessage>(in_bg_);

    if(!fg_msg->hasChannels(1, CV_8U)) {
        throw std::runtime_error("foreground mask is not mono");
    }
    if(!bg_msg->hasChannels(1, CV_8U)) {
        throw std::runtime_error("background mask is not mono");
    }

    const cv::Mat& img = img_msg->value;
    const cv::Mat& fg_certain = fg_msg->value;
    const cv::Mat& bg_certain = bg_msg->value;

    bool has_fg = false;
    bool has_bg = false;
    cv::Mat mask(img.rows, img.cols, CV_8UC1, cv::Scalar::all(readParameter<int>("initial value")));

    {
        INTERLUDE("create mask");

        unsigned char threshold = readParameter<int>("threshold");
        for(int i = 0; i < img.rows; ++i) {
            for(int j = 0; j < img.rows; ++j) {
                if(fg_certain.at<unsigned char>(i,j) > threshold) {
                    mask.at<unsigned char>(i,j) = cv::GC_FGD;
                    has_fg = true;
                } else if(bg_certain.at<unsigned char>(i,j) > threshold) {
                    mask.at<unsigned char>(i,j) = cv::GC_BGD;
                    has_bg = true;
                }
            }
        }

        if(!has_fg || !has_bg) {
            CvMatMessage::Ptr bg_out(new CvMatMessage(enc::mono, img_msg->stamp_micro_seconds));
            CvMatMessage::Ptr fg_out(new CvMatMessage(enc::mono, img_msg->stamp_micro_seconds));
            bg_out->value = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar::all(255));
            fg_out->value = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar::all(0));
            msg::publish(out_fg_, fg_out);
            msg::publish(out_bg_, bg_out);
            return;
        }


    }

    cv::Rect roi;
    if(msg::hasMessage(in_roi_)) {
        roi = msg::getMessage<RoiMessage>(in_roi_)->value.rect();
    } else {
        roi = cv::Rect(0, 0, img.cols, img.rows);
    }

    // todo: allow state to be remembered
    int iter = readParameter<int>("iterations");
    int mode = cv::GC_INIT_WITH_MASK;
    cv::Mat fg, bg;

    {
        INTERLUDE("GrabCut");
        cv::grabCut(img, mask, roi, bg, fg, iter, mode);
    }

    CvMatMessage::Ptr bg_out(new CvMatMessage(enc::mono, img_msg->stamp_micro_seconds));
    CvMatMessage::Ptr fg_out(new CvMatMessage(enc::mono, img_msg->stamp_micro_seconds));

    bg_out->value = 255 - (mask & 1);
    fg_out->value = mask & 1;

    msg::publish(out_fg_, fg_out);
    msg::publish(out_bg_, bg_out);
}

