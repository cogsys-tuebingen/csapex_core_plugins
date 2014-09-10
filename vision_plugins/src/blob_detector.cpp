/// HEADER
#include "blob_detector.h"

/// COMPONENT
#include <vision_plugins/cvblob.h>

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex_vision_features/keypoint_message.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <sstream>

CSAPEX_REGISTER_CLASS(csapex::BlobDetector, csapex::Node)

using namespace csapex;
using namespace connection_types;
using namespace cvb;

BlobDetector::BlobDetector()
{
    addParameter(param::ParameterFactory::declareBool("RoiInformation",
                                               param::ParameterDescription("Show the information of each RoI"),
                                               false));

}

BlobDetector::~BlobDetector()
{
}

#define _HSV2RGB_(H, S, V, R, G, B) \
{ \
  double _h = H/60.; \
  int _hf = (int)floor(_h); \
  int _hi = ((int)_h)%6; \
  double _f = _h - _hf; \
  \
  double _p = V * (1. - S); \
  double _q = V * (1. - _f * S); \
  double _t = V * (1. - (1. - _f) * S); \
  \
  switch (_hi) \
  { \
    case 0: \
        R = 255.*V; G = 255.*_t; B = 255.*_p; \
    break; \
    case 1: \
        R = 255.*_q; G = 255.*V; B = 255.*_p; \
    break; \
    case 2: \
        R = 255.*_p; G = 255.*V; B = 255.*_t; \
    break; \
    case 3: \
        R = 255.*_p; G = 255.*_q; B = 255.*V; \
    break; \
    case 4: \
        R = 255.*_t; G = 255.*_p; B = 255.*V; \
    break; \
    case 5: \
        R = 255.*V; G = 255.*_p; B = 255.*_q; \
    break; \
  } \
}

void BlobDetector::process()
{
    bool roi_info = readParameter<bool>("RoiInformation");

    CvMatMessage::Ptr img = input_->getMessage<CvMatMessage>();

    if(img->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    cv::Mat& gray = img->value;

    CvMatMessage::Ptr debug(new CvMatMessage(enc::bgr));
    cv::cvtColor(gray, debug->value, CV_GRAY2BGR);

    CvBlobs blobs;

    IplImage* grayPtr = new IplImage(gray);
    IplImage* labelImgPtr = cvCreateImage(cvGetSize(grayPtr), IPL_DEPTH_LABEL, 1);

    cvLabel(grayPtr, labelImgPtr, blobs);


    VectorMessage::Ptr out(VectorMessage::make<RoiMessage>());

    for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it) {
        const CvBlob& blob = *it->second;

        RoiMessage::Ptr roi(new RoiMessage);
        double r, g, b;
        _HSV2RGB_((double)((blob.label *77)%360), .5, 1., r, g, b);
        cv::Scalar color(b,g,r);
        roi->value = Roi(blob.minx, blob.miny, (blob.maxx - blob.minx + 1), (blob.maxy - blob.miny + 1), color);

        out->value.push_back(roi);
    }

    output_->publish(out);


    if(output_debug_->isConnected()) {
        IplImage* debugPtr = new IplImage(debug->value);
        cvRenderBlobs(labelImgPtr, blobs, debugPtr, debugPtr);

        for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it) {
            const CvBlob& blob = *it->second;
            const CvChainCodes& c = blob.contour.chainCode;

            cv::Point p = blob.contour.startingPoint;
            for(CvChainCodes::const_iterator i = c.begin(); i != c.end(); ++i) {
                const CvChainCode& chain = *i;
                cv::Point next = p + cv::Point(cvChainCodeMoves[chain][0], cvChainCodeMoves[chain][1]);

                double r, g, b;
                _HSV2RGB_((double)((blob.label *77)%360), .5, 1., r, g, b);
                cv::Scalar color(b,g,r);
                cv::line(debug->value, p, next, color, 5, CV_AA);

                p = next;
            }
        }

        if (roi_info){
            for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it) {
                const CvBlob& blob = *it->second;
                std::stringstream ss;
                ss << "Blob #" << blob.label << ": A=" << blob.area << ", C=(" << blob.centroid.x << ", " << blob.centroid.y << ")";

                double r, g, b;
                _HSV2RGB_((double)((blob.label *77)%360), .5, 1., r, g, b);
                cv::Scalar color(b,g,r);

                cv::putText(debug->value, ss.str(), cv::Point(blob.centroid.x, blob.centroid.y), cv::FONT_HERSHEY_PLAIN, 1.0, color, 3);
                cv::putText(debug->value, ss.str(), cv::Point(blob.centroid.x, blob.centroid.y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(255), 1);
            }
        }
        output_debug_->publish(debug);
    }


    cvReleaseImage(&labelImgPtr);
}


void BlobDetector::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("Image");

    output_debug_ = modifier_->addOutput<CvMatMessage>("OutputImage");
    output_ = modifier_->addOutput<VectorMessage, RoiMessage>("ROIs");
}
