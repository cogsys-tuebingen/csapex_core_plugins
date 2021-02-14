/// HEADER
#include "blob_detector.h"

/// COMPONENT
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <csapex_opencv/cvblob.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format off

/// PROJECT
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <sstream>

CSAPEX_REGISTER_CLASS(csapex::BlobDetector, csapex::Node)

using namespace csapex;
using namespace connection_types;
using namespace cvb;

BlobDetector::BlobDetector()
{
}

BlobDetector::~BlobDetector()
{
}

void BlobDetector::setupParameters(Parameterizable& parameters)
{
    addParameter(csapex::param::factory::declareBool("RoiInformation", csapex::param::ParameterDescription("Show the information of each RoI"), false));

    addParameter(csapex::param::factory::declareInterval("Area", csapex::param::ParameterDescription("Area for the reduced image"), 1, 800000, 1, 800000, 1));
}

#define _HSV2RGB_(H, S, V, R, G, B)                                                                                                                                                                    \
    {                                                                                                                                                                                                  \
        double _h = H / 60.;                                                                                                                                                                           \
        int _hf = (int)floor(_h);                                                                                                                                                                      \
        int _hi = ((int)_h) % 6;                                                                                                                                                                       \
        double _f = _h - _hf;                                                                                                                                                                          \
                                                                                                                                                                                                       \
        double _p = V * (1. - S);                                                                                                                                                                      \
        double _q = V * (1. - _f * S);                                                                                                                                                                 \
        double _t = V * (1. - (1. - _f) * S);                                                                                                                                                          \
                                                                                                                                                                                                       \
        switch (_hi) {                                                                                                                                                                                 \
            case 0:                                                                                                                                                                                    \
                R = 255. * V;                                                                                                                                                                          \
                G = 255. * _t;                                                                                                                                                                         \
                B = 255. * _p;                                                                                                                                                                         \
                break;                                                                                                                                                                                 \
            case 1:                                                                                                                                                                                    \
                R = 255. * _q;                                                                                                                                                                         \
                G = 255. * V;                                                                                                                                                                          \
                B = 255. * _p;                                                                                                                                                                         \
                break;                                                                                                                                                                                 \
            case 2:                                                                                                                                                                                    \
                R = 255. * _p;                                                                                                                                                                         \
                G = 255. * V;                                                                                                                                                                          \
                B = 255. * _t;                                                                                                                                                                         \
                break;                                                                                                                                                                                 \
            case 3:                                                                                                                                                                                    \
                R = 255. * _p;                                                                                                                                                                         \
                G = 255. * _q;                                                                                                                                                                         \
                B = 255. * V;                                                                                                                                                                          \
                break;                                                                                                                                                                                 \
            case 4:                                                                                                                                                                                    \
                R = 255. * _t;                                                                                                                                                                         \
                G = 255. * _p;                                                                                                                                                                         \
                B = 255. * V;                                                                                                                                                                          \
                break;                                                                                                                                                                                 \
            case 5:                                                                                                                                                                                    \
                R = 255. * V;                                                                                                                                                                          \
                G = 255. * _p;                                                                                                                                                                         \
                B = 255. * _q;                                                                                                                                                                         \
                break;                                                                                                                                                                                 \
        }                                                                                                                                                                                              \
    }

void BlobDetector::process()
{
    bool roi_info = readParameter<bool>("RoiInformation");

    CvMatMessage::ConstPtr img = msg::getMessage<CvMatMessage>(input_);

    if (!img->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    const cv::Mat& gray = img->value;

    CvMatMessage::Ptr debug(new CvMatMessage(enc::bgr, img->frame_id, img->stamp_micro_seconds));
    cv::cvtColor(gray, debug->value, cv::COLOR_GRAY2BGR);

    CvMatMessage::Ptr reduced(new CvMatMessage(enc::mono, img->frame_id, img->stamp_micro_seconds));
    reduced->value = cv::Mat::zeros(img->value.rows, img->value.cols, CV_8U);

    CvBlobs blobs;

#if CV_MAJOR_VERSION <= 3 && CV_MINOR_VERSION <= 3
    IplImage grayPtr(gray);
#else
    IplImage grayPtr(cvIplImage(gray));
#endif
    IplImage* labelImgPtr = cvCreateImage(cvGetSize(&grayPtr), IPL_DEPTH_LABEL, 1);

    cvLabel(&grayPtr, labelImgPtr, blobs);

    std::shared_ptr<std::vector<RoiMessage>> out(new std::vector<RoiMessage>);

    std::pair<unsigned int, unsigned int> range_area = readParameter<std::pair<int, int>>("Area");

    for (CvBlobs::iterator it = blobs.begin(); it != blobs.end();) {
        const CvBlob& blob = *it->second;

        int w = (blob.maxx - blob.minx + 1);
        int h = (blob.maxy - blob.miny + 1);

        if ((blob.area < range_area.first) || (blob.area > range_area.second)) {
            cvReleaseBlob((*it).second);
            it = blobs.erase(it);
            continue;
        }

        ++it;

        RoiMessage roi;
        double r, g, b;
        _HSV2RGB_((double)((blob.label * 77) % 360), .5, 1., r, g, b);
        cv::Scalar color(b, g, r);

        roi.value = Roi(blob.minx, blob.miny, w, h, color);
        out->push_back(roi);
    }

    //    ainfo << blobs.size() << " blobs" << std::endl;

    msg::publish<GenericVectorMessage, RoiMessage>(output_, out);

    if (msg::isConnected(output_debug_)) {
#if CV_MAJOR_VERSION <= 3 && CV_MINOR_VERSION <= 3
        IplImage debugPtr(debug->value);
#else
        IplImage debugPtr(cvIplImage(debug->value));
#endif
        cvRenderBlobs(labelImgPtr, blobs, &debugPtr, &debugPtr);

        for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
            const CvBlob& blob = *it->second;
            const CvChainCodes& c = blob.contour.chainCode;

            cv::Point p = blob.contour.startingPoint;
            for (CvChainCodes::const_iterator i = c.begin(); i != c.end(); ++i) {
                const CvChainCode& chain = *i;
                cv::Point next = p + cv::Point(cvChainCodeMoves[chain][0], cvChainCodeMoves[chain][1]);

                double r, g, b;
                _HSV2RGB_((double)((blob.label * 77) % 360), .5, 1., r, g, b);
                cv::Scalar color(b, g, r);
                cv::line(debug->value, p, next, color, 1, cv::LINE_AA);

                p = next;
            }
        }

        if (roi_info) {
            for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
                const CvBlob& blob = *it->second;
                std::stringstream ss;
                ss << "Blob #" << blob.label << ": A=" << blob.area << ", C=(" << blob.centroid.x << ", " << blob.centroid.y << ")";

                double r, g, b;
                _HSV2RGB_((double)((blob.label * 77) % 360), .5, 1., r, g, b);
                cv::Scalar color(b, g, r);

                cv::putText(debug->value, ss.str(), cv::Point(blob.centroid.x, blob.centroid.y), cv::FONT_HERSHEY_PLAIN, 1.0, color, 3);
                cv::putText(debug->value, ss.str(), cv::Point(blob.centroid.x, blob.centroid.y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(255), 1);
            }
        }
        msg::publish(output_debug_, debug);
    }

    if (msg::isConnected(output_reduce_)) {
#if CV_MAJOR_VERSION <= 3 && CV_MINOR_VERSION <= 3
        IplImage reducedPtr(reduced->value);
#else
        IplImage reducedPtr(cvIplImage(reduced->value));
#endif

        cvFilterByArea(blobs, range_area.first, range_area.second);

        cvFilterLabels(labelImgPtr, &reducedPtr, blobs);

        msg::publish(output_reduce_, reduced);
    }

    cvReleaseBlobs(blobs);
    cvReleaseImage(&labelImgPtr);
}

void BlobDetector::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("Image");

    output_debug_ = node_modifier.addOutput<CvMatMessage>("OutputImage");
    output_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("ROIs");
    output_reduce_ = node_modifier.addOutput<CvMatMessage>("ReducedImage");
}
