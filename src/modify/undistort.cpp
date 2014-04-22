/// HEADER
#include "undistort.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(vision_plugins::Undistort, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

Undistort::Undistort()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    addParameter(param::ParameterFactory::declareFileInputPath("file", ""), boost::bind(&Undistort::update, this));
    addParameter(param::ParameterFactory::declareRange("margin", 0, 1000, 0, 1), boost::bind(&Undistort::update, this));

    std::vector< std::pair<std::string, int> > modes;
    modes.push_back(std::make_pair("nearest", (int) CV_INTER_NN));
    modes.push_back(std::make_pair("linear", (int) CV_INTER_LINEAR));
    modes.push_back(std::make_pair("area", (int) CV_INTER_AREA));
    modes.push_back(std::make_pair("cubic", (int) CV_INTER_CUBIC));
    modes.push_back(std::make_pair("lanczos4", (int) CV_INTER_LANCZOS4));
    addParameter(param::ParameterFactory::declareParameterSet<int>("mode", modes), boost::bind(&Undistort::update, this));

}

void Undistort::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    out->value = in->value.clone();
    if(undist_.get() != NULL) {
        int margin       = param<int>("margin");
        cv::Size  margin_size(2 * margin + in->value.cols, 2 * margin + in->value.rows);
        undist_->reset_map(margin_size, margin, margin);
        undist_->undistort(out->value, out->value);
    }
    output_->publish(out);
}

void Undistort::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Distorted");
    output_ = addOutput<CvMatMessage>("Undistorted");

    update();
}

bool Undistort::read_matrices(const std::string &path, cv::Mat &intrinsics, cv::Mat &distortion_coeffs)
{
    if(path == "")
        return false;

    try {
        cv::FileStorage fs(path, cv::FileStorage::READ);

        if(!fs.isOpened())
            return false;

        fs["intrinsics"] >> intrinsics;
        fs["distortion"] >> distortion_coeffs;

        return true;
    } catch(cv::Exception e) {
        return false;
    }
}

void Undistort::update()
{

    std::string path = param<std::string>("file");
    cv::Mat intr;
    cv::Mat coef;
    int mode = param<int>("mode");

    if(read_matrices(path, intr, coef)) {
        undist_.reset(new utils_cv::Undistortion(intr, coef, mode));
    }

}
