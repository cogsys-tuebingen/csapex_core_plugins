/// HEADER
#include "undistort.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::Undistort, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;
using namespace csapex;

Undistort::Undistort()
{
}

void Undistort::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(in->getEncoding(), in->frame_id, in->stamp_micro_seconds));

    out->value = in->value.clone();
    if(!undist_.empty()) {
        int margin       = readParameter<int>("margin");
        cv::Size  margin_size(2 * margin + in->value.cols, 2 * margin + in->value.rows);
        undist_->reset_map(margin_size, margin, margin);
        undist_->undistort(out->value, out->value);
    }
    msg::publish(output_, out);
}

void Undistort::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("undistorted");

    update();
}

void Undistort::setupParameters(Parameterizable& parameters)
{

    parameters.addParameter(csapex::param::ParameterFactory::declareBool("load from file",
                                                                         param::ParameterDescription("Load a calibration from a .yaml file."),
                                                                         true),
                            std::bind(&Undistort::update, this));

    auto load_from_file = [this](){return readParameter<bool>("load from file");};
    auto do_not_load_from_file = [this](){return !readParameter<bool>("load from file");};

    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareFileInputPath("file",
                                                                                             param::ParameterDescription("Path to .yaml calibration file."),
                                                                                             "", "*.yaml"),
                                       load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("fx", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("fy", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("cx", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("cy", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("k1", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("k2", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("k3", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("k4", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("k5", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("k6", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("p1", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareValue("p2", 0.0),
                                       do_not_load_from_file,
                                       std::bind(&Undistort::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("margin",
                                                                          param::ParameterDescription("Add a margin to the undistorted image."),
                                                                          0, 1000, 0, 1),
                            std::bind(&Undistort::update, this));

    std::map<std::string, int> modes;
    modes["nearest"] = (int) CV_INTER_NN;
    modes["linear"] = (int) CV_INTER_LINEAR;
    modes["area"] = (int) CV_INTER_AREA;
    modes["cubic"] = (int) CV_INTER_CUBIC;
    modes["lanczos4"] = (int) CV_INTER_LANCZOS4;
    parameters.addParameter(csapex::param::ParameterFactory::declareParameterSet<int>("mode", modes, (int) CV_INTER_NN), std::bind(&Undistort::update, this));
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
    cv::Mat intr;
    cv::Mat coef;
    if(readParameter<bool>("load from file")) {
        std::string path = readParameter<std::string>("file");
        if(!read_matrices(path, intr, coef)) {
            aerr << "Could not read file!" << std::endl;
            return;
        }
        ainfo << "Successfully read file!" << std::endl;
    } else {
        intr = cv::Mat::eye(3,3,CV_64FC1);
        coef = cv::Mat::zeros(8,1, CV_64FC1);
        intr.at<double>(0,0) = readParameter<double>("fx");
        intr.at<double>(1,1) = readParameter<double>("fy");
        intr.at<double>(0,2) = readParameter<double>("cx");
        intr.at<double>(1,2) = readParameter<double>("cy");
        coef.at<double>(0)   = readParameter<double>("k1");
        coef.at<double>(1)   = readParameter<double>("k2");
        coef.at<double>(2)   = readParameter<double>("p1");
        coef.at<double>(3)   = readParameter<double>("p2");
        coef.at<double>(4)   = readParameter<double>("k3");
        coef.at<double>(5)   = readParameter<double>("k4");
        coef.at<double>(6)   = readParameter<double>("k5");
        coef.at<double>(7)   = readParameter<double>("k6");
    }

    int mode = readParameter<int>("mode");
    undist_ = cslibs_vision::Undistortion::Ptr(new cslibs_vision::Undistortion(intr, coef, mode));
}
