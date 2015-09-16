/// HEADER
#include "vj_detector.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <fstream>
#include <CascadeDetector.h>

CSAPEX_REGISTER_CLASS(csapex::VJDetector, csapex::Node)

using namespace csapex;
using namespace connection_types;

VJDetector::VJDetector()
    : vj_detector(nullptr), image_scanner(nullptr)
{
    addParameter(csapex::param::ParameterFactory::declareFileInputPath("file", ""));
}

VJDetector::~VJDetector()
{
    if(vj_detector) {
        delete vj_detector;
    }
    if(image_scanner) {
        delete image_scanner;
    }
}

void VJDetector::process()
{
    std::string filename = readParameter<std::string>("file");

    if(!vj_detector || filename != file_) {
        file_ = filename;

        if(vj_detector) {
            delete vj_detector;
            vj_detector = nullptr;
            delete image_scanner;
            image_scanner = nullptr;
        }

        if(!file_.empty()) {
            std::ifstream in(file_.c_str());
            if(!in.is_open()) {
                throw std::runtime_error(std::string("Error Viola Jones Classifier from file: ") + file_);

            } else {
                ainfo << "loading classifier from " << file_ << std::endl;

                vj_detector = CascadeDetector::load(&in, false, false);
                image_scanner = new ImageScanner(vj_detector,
                                                 7.5, // minscale
                                                 55.0, // maxscale
                                                 1.414213562);
            }
        }
    }

    if(!vj_detector) {
        return;
    }

    CvMatMessage::ConstPtr a = msg::getMessage<CvMatMessage>(input_);

    if(!a->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    VectorMessage::Ptr out(VectorMessage::make<RoiMessage>());

    QImage img(a->value.data, a->value.cols, a->value.rows, QImage::Format_Indexed8);
    image_scanner->setImage(&img);

    utilities::rectangle rect;
    while(image_scanner->scanNext(&rect)) {
        RoiMessage::Ptr msg(new RoiMessage);
        msg->value = cv::Rect(rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top);
        out->value.push_back(msg);
    }

    msg::publish(output_, out);
}


void VJDetector::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("Image");

    output_ = modifier_->addOutput<VectorMessage, RoiMessage>("ROIs");
}
