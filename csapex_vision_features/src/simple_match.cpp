/// HEADER
#include "simple_match.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils/matcher.h>
#include <data/matchable.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <opencv2/opencv.hpp>

CSAPEX_REGISTER_CLASS(csapex::SimpleMatch, csapex::Node)

using namespace csapex;
using namespace connection_types;

class MatchableImpl : public Matchable
{
public:
    /**
     * @brief Matchable
     * @param keypoints
     * @param descriptors
     */
    MatchableImpl(const std::vector<cv::KeyPoint> &keypoints, const cv::Mat& descriptors)
        : Matchable(keypoints, descriptors)
    {

    }
    /**
     * @brief get_dimensions
     * @return the width and height of this matchable
     */
    virtual cv::Rect getDimensions() const
    {
        return cv::Rect(0,0,0,0);
    }
};

SimpleMatch::SimpleMatch()
    : in_img_1(NULL)
{
    addTag(Tag::get("Features"));
}


void SimpleMatch::process()
{
    CvMatMessage::Ptr img1 = in_img_1->getMessage<CvMatMessage>();
    CvMatMessage::Ptr img2 = in_img_2->getMessage<CvMatMessage>();

    KeypointMessage::Ptr key1 = in_key_1->getMessage<KeypointMessage>();
    KeypointMessage::Ptr key2 = in_key_2->getMessage<KeypointMessage>();

    DescriptorMessage::Ptr des1 = in_des_1->getMessage<DescriptorMessage>();
    DescriptorMessage::Ptr des2 = in_des_2->getMessage<DescriptorMessage>();


    if(des1->value.type() != des2->value.type()) {
        setError(true, "#types don't match");
        return;
    }

    std::vector<std::vector<cv::DMatch> > matches;

    if(des1->value.cols > 0 && des2->value.cols > 0) {
        MatchableImpl m1(key1->value, des1->value);
        MatchableImpl m2(key2->value, des2->value);

        Matcher m(des1->isBinary());
        m.match(&m1, &m2, matches);
    }

    CvMatMessage::Ptr out(new CvMatMessage(img1->getEncoding()));

    cv::Scalar matchColor = cv::Scalar::all(255);
    cv::Scalar singlePointColor = cv::Scalar(192, 64, 64);
    std::vector<std::vector<char> > mask;
    int flag = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
    cv::drawMatches(img1->value, key1->value, img2->value, key2->value, matches, out->value, matchColor, singlePointColor, mask, flag);
    out_img->publish(out);
}


void SimpleMatch::setup()
{
    setSynchronizedInputs(true);

    in_img_1 = addInput<CvMatMessage>("Image 1");
    in_key_1 = addInput<KeypointMessage>("Keypoints 1");
    in_des_1 = addInput<DescriptorMessage>("Descriptor 1");

    in_img_2 = addInput<CvMatMessage>("Image 2");
    in_key_2 = addInput<KeypointMessage>("Keypoints 2");
    in_des_2 = addInput<DescriptorMessage>("Descriptor 2");

    out_img = addOutput<CvMatMessage>("Debug View");
}
