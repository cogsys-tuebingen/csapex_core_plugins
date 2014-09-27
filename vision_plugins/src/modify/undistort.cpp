/// HEADER
#include "undistort.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(vision_plugins::Undistort, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

Undistort::Undistort()
{
}

void Undistort::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding()));

    out->value = in->value.clone();
    if(undist_.get() != NULL) {
        int margin       = readParameter<int>("margin");
        cv::Size  margin_size(2 * margin + in->value.cols, 2 * margin + in->value.rows);
        undist_->reset_map(margin_size, margin, margin);
        undist_->undistort(out->value, out->value);
    }
    output_->publish(out);
}

void Undistort::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("original");
    output_ = modifier_->addOutput<CvMatMessage>("undistorted");

    update();
}

void Undistort::setupParameters()
{
    addParameter(param::ParameterFactory::declareFileInputPath("file", ""), boost::bind(&Undistort::update, this));
    addParameter(param::ParameterFactory::declareRange("margin", 0, 1000, 0, 1), boost::bind(&Undistort::update, this));

    std::map<std::string, int> modes;
    modes["nearest"] = (int) CV_INTER_NN;
    modes["linear"] = (int) CV_INTER_LINEAR;
    modes["area"] = (int) CV_INTER_AREA;
    modes["cubic"] = (int) CV_INTER_CUBIC;
    modes["lanczos4"] = (int) CV_INTER_LANCZOS4;
    addParameter(param::ParameterFactory::declareParameterSet<int>("mode", modes, (int) CV_INTER_NN), boost::bind(&Undistort::update, this));
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

    std::string path = readParameter<std::string>("file");
    cv::Mat intr;
    cv::Mat coef;
    int mode = readParameter<int>("mode");

    if(read_matrices(path, intr, coef)) {
        undist_.reset(new utils_cv::Undistortion(intr, coef, mode));
    }

}
