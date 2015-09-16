/// HEADER
#include "find_homography.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/match_message.h>

#include <csapex/param/parameter_factory.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

CSAPEX_REGISTER_CLASS(csapex::FindHomography, csapex::Node)

using namespace csapex;
using namespace connection_types;

FindHomography::FindHomography()
    : in_key_1(nullptr), in_key_2(nullptr)
{

}

void FindHomography::setup(NodeModifier& node_modifier)
{
    in_img_1 = node_modifier.addInput<CvMatMessage>("Image 1");
    in_key_1 = node_modifier.addInput<KeypointMessage>("Keypoints 1");
    in_img_2 = node_modifier.addInput<CvMatMessage>("Image 2");
    in_key_2 = node_modifier.addInput<KeypointMessage>("Keypoints 2");
    in_matches = node_modifier.addInput<MatchMessage>("Matches");

    out_img = node_modifier.addOutput<CvMatMessage>("Debug View");
}


void FindHomography::setupParameters(Parameterizable &parameters)
{
    std::function<void(csapex::param::Parameter*)> update = std::bind(&FindHomography::update, this);

    std::map<std::string, int> methods = boost::assign::map_list_of
            ("Regular", (int) 0)
            ("RANSAC", (int) CV_RANSAC)
            ("LMedS", (int) CV_LMEDS);

    csapex::param::Parameter::Ptr method = csapex::param::ParameterFactory::declareParameterSet("method", methods, (int) 0);
    parameters.addParameter(method, update);
    std::function<bool()> cond_ransac = [this]() { return method_ == CV_RANSAC; };

    parameters.addConditionalParameter(csapex::param::ParameterFactory::declareRange("ransac/threshold", 0.5, 10., 3., 0.5), cond_ransac);
}

void FindHomography::update()
{
    method_ = readParameter<int>("method");
    ransac_threshold_ = readParameter<double>("ransac/threshold");
    std::cout << "ransac threshold is now: " << ransac_threshold_ << std::endl;
}

void FindHomography::process()
{
    KeypointMessage::ConstPtr keypoints1 = msg::getMessage<KeypointMessage>(in_key_1);
    KeypointMessage::ConstPtr keypoints2 = msg::getMessage<KeypointMessage>(in_key_2);

    CvMatMessage::ConstPtr image1 = msg::getMessage<CvMatMessage>(in_img_1);
    CvMatMessage::ConstPtr image2 = msg::getMessage<CvMatMessage>(in_img_2);

    MatchMessage::ConstPtr matches = msg::getMessage<MatchMessage>(in_matches);

    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    for (std::vector<cv::DMatch>::const_iterator it = matches->value.begin();
         it != matches->value.end(); it++) {
        points1.push_back(keypoints1->value[it->queryIdx].pt);
        points2.push_back(keypoints2->value[it->trainIdx].pt);
    }

    cv::Mat mask;
    cv::Mat H = cv::findHomography(points1, points2, mask, method_, ransac_threshold_);

    CvMatMessage::Ptr out(new CvMatMessage(image1->getEncoding(), image1->stamp_micro_seconds));

    cv::drawMatches(image1->value, keypoints1->value,
                    image2->value, keypoints2->value,
                    matches->value, out->value, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    // Transform corners of image 1 to image 2
    std::vector<cv::Point2f> corners1(4);
    corners1[0] = cvPoint(0,0);
    corners1[1] = cvPoint(image1->value.cols, 0 );
    corners1[2] = cvPoint(image1->value.cols, image1->value.rows );
    corners1[3] = cvPoint(0, image1->value.rows );
    std::vector<cv::Point2f> corners2(4);
    cv::perspectiveTransform(corners1, corners2, H);

    // Draw the transformed corners
    cv::line(out->value, corners2[0] + cv::Point2f(image1->value.cols, 0), corners2[1] + cv::Point2f(image1->value.cols, 0), cv::Scalar(0, 255, 0), 4 );
    cv::line(out->value, corners2[1] + cv::Point2f(image1->value.cols, 0), corners2[2] + cv::Point2f(image1->value.cols, 0), cv::Scalar(0, 255, 0), 4 );
    cv::line(out->value, corners2[2] + cv::Point2f(image1->value.cols, 0), corners2[3] + cv::Point2f(image1->value.cols, 0), cv::Scalar(0, 255, 0), 4 );
    cv::line(out->value, corners2[3] + cv::Point2f(image1->value.cols, 0), corners2[0] + cv::Point2f(image1->value.cols, 0), cv::Scalar(0, 255, 0), 4 );

    msg::publish(out_img, out);
}
