/// HEADER
#include "vj_detector.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <fstream>
#include <CascadeDetector.h>

CSAPEX_REGISTER_CLASS(csapex::VJDetector, csapex::Node)

using namespace csapex;
using namespace connection_types;

VJDetector::VJDetector()
    : vj_detector(NULL), image_scanner(NULL)
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("ROI"));

    addParameter(param::ParameterFactory::declarePath("file", ""));
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
    std::string filename = param<std::string>("file");

    if(!vj_detector || filename != file_) {
        file_ = filename;

        if(vj_detector) {
            delete vj_detector;
            vj_detector = NULL;
            delete image_scanner;
            image_scanner = NULL;
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

    CvMatMessage::Ptr a = input_->getMessage<CvMatMessage>();

    if(a->getEncoding() != enc::mono) {
        throw std::runtime_error("image must be grayscale.");
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

    output_->publish(out);
}


void VJDetector::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<CvMatMessage>("Image");

    output_ = addOutput<VectorMessage, RoiMessage>("ROIs");
}
