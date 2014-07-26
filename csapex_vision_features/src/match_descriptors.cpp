/// HEADER
#include "match_descriptors.h"

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_vision/utils/matcher.h>
#include <utils_vision/data/matchable.h>
#include <utils_vision/utils/hough_peak.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
#include <boost/assign/std.hpp>
#include <boost/lambda/lambda.hpp>

CSAPEX_REGISTER_CLASS(csapex::MatchDescriptors, csapex::Node)

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

//RobustMatcher class taken from OpenCV2 Computer Vision Application Programming Cookbook Ch 9
class RobustMatcher
{
private:
    // pointer to the matcher object
    cv::Ptr<cv::DescriptorMatcher > matcher;
    float ratio; // max ratio between 1st and 2nd NN
    bool refineF; // if true will refine the F matrix
    double confidence; // confidence level (probability)
    double distance; // min distance to epipolar
public:
    RobustMatcher(cv::Ptr<cv::DescriptorMatcher > m) : ratio(0.75f), refineF(true),
        confidence(0.99), distance(3.0) {
        matcher= m;
    }

    // Set the matcher
    void setDescriptorMatcher(
            cv::Ptr<cv::DescriptorMatcher>& match) {
        matcher= match;
    }
    // Set confidence level
    void setConfidenceLevel(
            double conf) {
        confidence= conf;
    }
    //Set MinDistanceToEpipolar
    void setMinDistanceToEpipolar(
            double dist) {
        distance= dist;
    }
    //Set ratio
    void setRatio(
            float rat) {
        ratio= rat;
    }

    // Clear matches for which NN ratio is > than threshold
    // return the number of removed points
    // (corresponding entries being cleared,
    // i.e. size will be 0)
    int ratioTest(std::vector<std::vector<cv::DMatch> >
                  &matches) {
        int removed=0;
        // for all matches
        for(std::vector<std::vector<cv::DMatch> >::iterator
            matchIterator= matches.begin();
            matchIterator!= matches.end(); ++matchIterator) {
            // if 2 NN has been identified
            if(matchIterator->size() > 1) {
                // check distance ratio
                if((*matchIterator)[0].distance/
                        (*matchIterator)[1].distance > ratio) {
                    matchIterator->clear(); // remove match
                    removed++;
                }
            } else { // does not have 2 neighbours
                matchIterator->clear(); // remove match
                removed++;
            }
        }
        return removed;
    }

    // Insert symmetrical matches in symMatches vector
    void symmetryTest(
            const std::vector<std::vector<cv::DMatch> >& matches1,
            const std::vector<std::vector<cv::DMatch> >& matches2,
            std::vector<cv::DMatch>& symMatches) {
        // for all matches image 1 -> image 2
        for(std::vector<std::vector<cv::DMatch> >::
            const_iterator matchIterator1= matches1.begin();
            matchIterator1!= matches1.end(); ++matchIterator1) {
            // ignore deleted matches
            if(matchIterator1->size() < 2)
                continue;
            // for all matches image 2 -> image 1
            for(std::vector<std::vector<cv::DMatch> >::
                const_iterator matchIterator2= matches2.begin();
                matchIterator2!= matches2.end();
                ++matchIterator2) {
                // ignore deleted matches
                if(matchIterator2->size() < 2)
                    continue;
                // Match symmetry test
                if((*matchIterator1)[0].queryIdx ==
                        (*matchIterator2)[0].trainIdx &&
                        (*matchIterator2)[0].queryIdx ==
                        (*matchIterator1)[0].trainIdx) {
                    // add symmetrical match
                    symMatches.push_back(
                                cv::DMatch((*matchIterator1)[0].queryIdx,
                                (*matchIterator1)[0].trainIdx,
                            (*matchIterator1)[0].distance));
                    break; // next match in image 1 -> image 2
                }
            }
        }
    }

    // Identify good matches using RANSAC
    // Return fundemental matrix
    cv::Mat ransacTest(
            const std::vector<cv::DMatch>& matches,
            const std::vector<cv::KeyPoint>& keypoints1,
            const std::vector<cv::KeyPoint>& keypoints2,
            std::vector<cv::DMatch>& outMatches) {
        // Convert keypoints into Point2f
        std::vector<cv::Point2f> points1, points2;
        cv::Mat fundemental;
        for(std::vector<cv::DMatch>::
            const_iterator it= matches.begin();
            it!= matches.end(); ++it) {
            // Get the position of left keypoints
            float x= keypoints1[it->queryIdx].pt.x;
            float y= keypoints1[it->queryIdx].pt.y;
            points1.push_back(cv::Point2f(x,y));
            // Get the position of right keypoints
            x= keypoints2[it->trainIdx].pt.x;
            y= keypoints2[it->trainIdx].pt.y;
            points2.push_back(cv::Point2f(x,y));
        }
        // Compute F matrix using RANSAC
        std::vector<uchar> inliers(points1.size(),0);
        if(points1.size()>0&&points2.size()>0) {
            cv::Mat fundemental= cv::findFundamentalMat(
                        cv::Mat(points1),cv::Mat(points2), // matching points
                        inliers,       // match status (inlier or outlier)
                        CV_FM_RANSAC, // RANSAC method
                        distance,      // distance to epipolar line
                        confidence); // confidence probability
            // extract the surviving (inliers) matches
            std::vector<uchar>::const_iterator
                    itIn= inliers.begin();
            std::vector<cv::DMatch>::const_iterator
                    itM= matches.begin();
            // for all matches
            for(; itIn!= inliers.end(); ++itIn, ++itM) {
                if(*itIn) {  // it is a valid match
                    outMatches.push_back(*itM);
                }
            }
            if(refineF) {
                // The F matrix will be recomputed with
                // all accepted matches
                // Convert keypoints into Point2f
                // for final F computation
                points1.clear();
                points2.clear();
                for(std::vector<cv::DMatch>::
                    const_iterator it= outMatches.begin();
                    it!= outMatches.end(); ++it) {
                    // Get the position of left keypoints
                    float x= keypoints1[it->queryIdx].pt.x;
                    float y= keypoints1[it->queryIdx].pt.y;
                    points1.push_back(cv::Point2f(x,y));
                    // Get the position of right keypoints
                    x= keypoints2[it->trainIdx].pt.x;
                    y= keypoints2[it->trainIdx].pt.y;
                    points2.push_back(cv::Point2f(x,y));
                }
                // Compute 8-point F from all accepted matches
                if(points1.size()>0&&points2.size()>0) {
                    fundemental= cv::findFundamentalMat(
                                cv::Mat(points1),cv::Mat(points2), // matches
                                CV_FM_8POINT); // 8-point method
                }
            }
        }
        return fundemental;
    }

    // Match feature points using symmetry test and RANSAC
    // returns fundemental matrix
    cv::Mat match(const cv::Mat& image1,
                  const cv::Mat& image2,
                  std::vector<cv::DMatch>& matches,
                  std::vector<cv::KeyPoint>& keypoints1,
                  std::vector<cv::KeyPoint>& keypoints2,
                  cv::Mat& descriptors1,
                  cv::Mat& descriptors2) {
        // 2. Match the two image descriptors
        // Construction of the matcher
        //cv::BruteForceMatcher<cv::L2<float>> matcher;
        // from image 1 to image 2
        // based on k nearest neighbours (with k=2)
        //        aerr << "d1: \n";
        //        for(size_t r = 0; r < descriptors1.rows; ++r) {
        //            for(size_t c = 0; c < descriptors1.cols; ++c) {
        //                aerr << descriptors1.at<float>(r,c) <<"\t";
        //            }
        //            aerr << "\n";
        //        }

        //        aerr << std::endl;


        //        aerr << "d2: \n";
        //        for(size_t r = 0; r < descriptors2.rows; ++r) {
        //            for(size_t c = 0; c < descriptors2.cols; ++c) {
        //                aerr << descriptors2.at<float>(r,c) <<"\t";
        //            }
        //            aerr << "\n";
        //        }

        //        aerr << std::endl;

        std::vector<std::vector<cv::DMatch> > matches1;

        matcher->knnMatch(descriptors1,descriptors2,
                          matches1, // vector of matches (up to 2 per entry)
                          2);        // return 2 nearest neighbours

        // from image 2 to image 1
        // based on k nearest neighbours (with k=2)
        std::vector<std::vector<cv::DMatch> > matches2;
        matcher->knnMatch(descriptors2,descriptors1,
                          matches2, // vector of matches (up to 2 per entry)
                          2);        // return 2 nearest neighbours
        // 3. Remove matches for which NN ratio is
        // > than threshold
        // clean image 1 -> image 2 matches
        // clean image 2 -> image 1 matches
        ratioTest(matches1);
        ratioTest(matches2);

        // 4. Remove non-symmetrical matches
        std::vector<cv::DMatch> symMatches;
        symmetryTest(matches1,matches2,symMatches);

        // 5. Validate matches using RANSAC
        cv::Mat fundemental= ransacTest(symMatches,
                                        keypoints1, keypoints2, matches);
        // return the found fundemental matrix
        return fundemental;
    }
};

MatchDescriptors::MatchDescriptors()
    : in_img_1(NULL), current_method_(SIMPLE)
{
    boost::function<void(param::Parameter*)> update = boost::bind(&MatchDescriptors::update, this);

    std::map<std::string, int> methods = boost::assign::map_list_of
            ("Simple", (int) SIMPLE)
            ("Peak", (int) PEAK)
            ("Robust", (int) ROBUST);

    param::Parameter::Ptr method = param::ParameterFactory::declareParameterSet("method", methods, (int) ROBUST);
    addParameter(method, update);

    addParameter(param::ParameterFactory::declareColorParameter("color/match", 255,128,128));
    addParameter(param::ParameterFactory::declareColorParameter("color/single", 128,128,255));

    // peak
    boost::function<bool()> cond_peak = (boost::bind(&param::Parameter::as<int>, method.get()) == PEAK);

    addConditionalParameter(param::ParameterFactory::declareRange("peak/cluster_count", 1, 32, 1, 1), cond_peak);
    addConditionalParameter(param::ParameterFactory::declareRange("peak/scaling", 1, 8, 1, 1), cond_peak);
    addConditionalParameter(param::ParameterFactory::declareRange("peak/octaves", 1, 12, 1, 1), cond_peak);
    addConditionalParameter(param::ParameterFactory::declareRange("peak/min_cluster_size", 1, 256, 1, 1), cond_peak);
}

void MatchDescriptors::update()
{
    current_method_ = static_cast<Method> (readParameter<int>("method"));
}

void MatchDescriptors::process()
{
    CvMatMessage::Ptr image1 = in_img_1->getMessage<CvMatMessage>();
    CvMatMessage::Ptr image2 = in_img_2->getMessage<CvMatMessage>();

    KeypointMessage::Ptr keypoints1 = in_key_1->getMessage<KeypointMessage>();
    KeypointMessage::Ptr keypoints2 = in_key_2->getMessage<KeypointMessage>();

    DescriptorMessage::Ptr descriptors1 = in_des_1->getMessage<DescriptorMessage>();
    DescriptorMessage::Ptr descriptors2 = in_des_2->getMessage<DescriptorMessage>();


    if(descriptors1->value.type() != descriptors2->value.type()) {
        setError(true, "#types don't match");
        return;
    }

    std::vector<std::vector<cv::DMatch> > matches;

    if(descriptors1->value.cols > 0 && descriptors2->value.cols > 0) {
        match(image1, image2, keypoints1, keypoints2, descriptors1, descriptors2, matches);
    }

    CvMatMessage::Ptr out(new CvMatMessage(image1->getEncoding()));

    std::vector<int> c;
    c = readParameter<std::vector<int> >("color/match");
    cv::Scalar matchColor  = cv::Scalar(c[2], c[1], c[0]);
    c = readParameter<std::vector<int> >("color/single");
    cv::Scalar singlePointColor  = cv::Scalar(c[2], c[1], c[0]);

    std::vector<std::vector<char> > mask;
    int flag = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;

    cv::drawMatches(image1->value, keypoints1->value, image2->value, keypoints2->value, matches, out->value, matchColor, singlePointColor, mask, flag);

    out_img->publish(out);
}

void MatchDescriptors::match(CvMatMessage::Ptr image1,
                             CvMatMessage::Ptr image2,
                             KeypointMessage::Ptr keypoints1,
                             KeypointMessage::Ptr keypoints2,
                             DescriptorMessage::Ptr descriptors1,
                             DescriptorMessage::Ptr descriptors2,
                             std::vector<std::vector<cv::DMatch> > &matches)
{
    switch(current_method_) {
    case SIMPLE:
        matchSimple(image1, image2, keypoints1, keypoints2, descriptors1, descriptors2, matches);
        break;
    case PEAK:
        matchPeak(image1, image2, keypoints1, keypoints2, descriptors1, descriptors2, matches);
        break;
    case ROBUST:
        matchRobust(image1, image2, keypoints1, keypoints2, descriptors1, descriptors2, matches);
        break;
    }
}

void MatchDescriptors::matchRobust(CvMatMessage::Ptr image1,
                                   CvMatMessage::Ptr image2,
                                   KeypointMessage::Ptr keypoints1,
                                   KeypointMessage::Ptr keypoints2,
                                   DescriptorMessage::Ptr descriptors1,
                                   DescriptorMessage::Ptr descriptors2,
                                   std::vector<std::vector<cv::DMatch> > &matches)
{
    cv::Ptr<cv::DescriptorMatcher > matcher;
    if(descriptors1->value.type() == CV_8U) {
        matcher = new cv::BFMatcher(cv::NORM_HAMMING);
    } else {
        matcher = new cv::BFMatcher(cv::NORM_L2);
    }
    RobustMatcher m(matcher);

    std::vector<cv::DMatch> tmp_matches;

    m.match(image1->value, image2->value, tmp_matches, keypoints1->value, keypoints2->value, descriptors1->value, descriptors2->value);

    for(std::size_t i = 0, n = tmp_matches.size(); i < n; ++i) {
        std::vector<cv::DMatch> v;
        v.push_back(tmp_matches[i]);
        matches.push_back(v);
    }
}


void MatchDescriptors::matchSimple(CvMatMessage::Ptr image1,
                                   CvMatMessage::Ptr image2,
                                   KeypointMessage::Ptr keypoints1,
                                   KeypointMessage::Ptr keypoints2,
                                   DescriptorMessage::Ptr descriptors1,
                                   DescriptorMessage::Ptr descriptors2,
                                   std::vector<std::vector<cv::DMatch> > &matches)
{
    MatchableImpl m1(keypoints1->value, descriptors1->value);
    MatchableImpl m2(keypoints2->value, descriptors2->value);

    Matcher m(descriptors1->isBinary());
    m.match(&m1, &m2, matches);
}




void MatchDescriptors::matchPeak(CvMatMessage::Ptr,
                                 CvMatMessage::Ptr,
                                 KeypointMessage::Ptr keypoints1,
                                 KeypointMessage::Ptr keypoints2,
                                 DescriptorMessage::Ptr descriptors1,
                                 DescriptorMessage::Ptr descriptors2,
                                 std::vector<std::vector<cv::DMatch> > &matches)
{
    MatchableImpl m1(keypoints1->value, descriptors1->value);
    MatchableImpl m2(keypoints2->value, descriptors2->value);

    HoughAlgorithm* h;

    if(false) {
        h = new HoughPeak<true, true> (readParameter<int>("peak/cluster_count"), readParameter<int>("peak/scaling"), readParameter<int>("peak/octaves"), m1, m2);
    } else {
        h = new HoughPeak<true, false>(readParameter<int>("peak/cluster_count"), readParameter<int>("peak/scaling"), readParameter<int>("peak/octaves"), m1, m2);
    }


    h->min_count = readParameter<int>("peak/min_cluster_size");

    std::vector<HoughData::Cluster> clusters;
    h->filter(matches, clusters);
}


void MatchDescriptors::setup()
{
    in_img_1 = modifier_->addInput<CvMatMessage>("Image 1");
    in_key_1 = modifier_->addInput<KeypointMessage>("Keypoints 1");
    in_des_1 = modifier_->addInput<DescriptorMessage>("Descriptor 1");

    in_img_2 = modifier_->addInput<CvMatMessage>("Image 2");
    in_key_2 = modifier_->addInput<KeypointMessage>("Keypoints 2");
    in_des_2 = modifier_->addInput<DescriptorMessage>("Descriptor 2");

    out_img = modifier_->addOutput<CvMatMessage>("Debug View");
}
