/// HEADER
#include "undistort.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

CSAPEX_REGISTER_CLASS(csapex::Undistort, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

Undistort::Undistort()
{
    addTag(Tag::get("Filter"));
    addTag(Tag::get("Vision"));

    addParameter(param::ParameterFactory::declarePath("file", ""), boost::bind(&Undistort::updateUndistorter, this));
    addParameter(param::ParameterFactory::declare("margin", 0, 1000, 0, 1), boost::bind(&Undistort::updateUndistorter, this));
}

void Undistort::allConnectorsArrived()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage);

    if(undist_.get() == NULL) {
        out->value = in->value.clone();
    } else {
        int margin       = param<int>("margin");
        cv::Size  margin_size(2 * margin + in->value.cols, 2 * margin + in->value.rows);
        undist_->reset_map(margin_size, margin, margin);

        cv::Mat working(margin_size.height, margin_size.width, in->value.type(), cv::Scalar::all(0));
        cv::Mat roi_working(working, cv::Rect(margin, margin, in->value.cols, in->value.rows));
        in->value.copyTo(roi_working);
        undist_->undistort(working, working);
        out->value = working;
    }
    output_->publish(out);
}

void Undistort::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Scan");
    output_ = addOutput<CvMatMessage>("Render");

    updateUndistorter();
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

void Undistort::updateUndistorter()
{

    std::string path = param<std::string>("file");
    cv::Mat intr;
    cv::Mat coef;

    if(read_matrices(path, intr, coef)) {
        undist_.reset(new Undistorter(intr, coef));
    }
}
