#include "keypoint_cluster.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision_features/keypoint_message.h>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

using namespace csapex;
using namespace connection_types;

// Register the class
PLUGINLIB_EXPORT_CLASS(csapex::KeypointCluster, csapex::BoxedObject)

using namespace csapex;

KeypointCluster::KeypointCluster()
    : in_keypoints_(NULL), out_(NULL), has_image_(false), has_keypoint_(false), publish_a_(true)
{
    // Insert this plug-in into the "Tutorial" category.
    // Create the category, if it doesn't exist.
    Tag::createIfNotExists("Vision");
    addTag(Tag::get("Vision"));
    eps_ = 5;
    minPts_ = 5;
    numberofrois_ = 1;
}

void KeypointCluster::fill(QBoxLayout *layout)
{
    // Connector for the first input image (sub id = 0)
    in_image_ = new ConnectorIn(box_, 0);
    in_image_->setLabel("Image");
    in_image_->setType(connection_types::CvMatMessage::make());

    in_keypoints_ = new ConnectorIn(box_, 1);
    in_keypoints_->setLabel("keypoints");
    in_keypoints_->setType(connection_types::KeypointMessage::make());

    // Connector for the output input image (sub id = 0)
    out_ = new ConnectorOut(box_, 2);
    out_->setLabel("Image");

    // Register the connectors
    box_->addInput(in_image_);
    box_->addInput(in_keypoints_);
    box_->addOutput(out_);

    epsilon_ = QtHelper::makeSlider(layout, "epsilon", 5, 1, 60);
    minpoints_ = QtHelper::makeSlider(layout, "Minpts", 5, 1, 40);

    opt = new QFrame;
    opt->setLayout(new QVBoxLayout);
    layout->addWidget(opt);

    QObject::connect(epsilon_, SIGNAL(valueChanged(int)), this , SLOT(updateeps(int)));
    QObject::connect(minpoints_, SIGNAL(valueChanged(int)), this , SLOT(updateminpts(int)));
}

void KeypointCluster::updateeps(int value){
    eps_ = value;
    updateroi(0);
}
void KeypointCluster::updateminpts(int value){
    minPts_ = value;
    updateroi(0);
}
void KeypointCluster::updateroislot(int value){
    chosenroi_ = value;
}

void KeypointCluster::messageArrived(ConnectorIn *source)
{
    // One of the two connectors has received a message, find out which
    if(source == in_image_) {
        has_image_ = true;
    } else if(source == in_keypoints_) {
        has_keypoint_ = true;
    }
    if(has_keypoint_ && has_image_) {
        has_keypoint_ = false;
        has_image_ = false;

        CvMatMessage::Ptr img_msg = boost::dynamic_pointer_cast<CvMatMessage> (in_image_->getMessage());

        ConnectionType::Ptr msg = in_keypoints_->getMessage();
        KeypointMessage::Ptr key_msg = boost::dynamic_pointer_cast<KeypointMessage>(msg);

        CvMatMessage::Ptr out(new CvMatMessage);

        std::vector<std::vector<cv::KeyPoint> > clusters = DBSCAN_keypoints(key_msg->value, eps_, minPts_);
        std::vector<cv::Mat> roivectors;

        numberofrois_ = clusters.size();
        int mostcluster = 0;
        int clster = 1;
        int max_x, min_x, max_y, min_y;
        int expander = 30;
        img_msg->value.copyTo(out->value);

        if(clusters.size() !=0){
            for(int k = 0; k < clusters.size() ;k++){
                max_x = 0;
                min_x = img_msg->value.cols;
                max_y = 0;
                min_y = img_msg->value.rows;
                cv::Scalar random = cv::Scalar((k*200)%255, (k*100)%255, (k*50)%255);
                if(clusters[k].size() > clster){
                    clster = clusters[k].size();
                    mostcluster = k;
                }
                for(int j = 0; j < clusters[k].size(); j++ ){
                    if(clusters[k].at(j).pt.x < min_x){
                        min_x = clusters[k].at(j).pt.x - expander;
                        if(min_x < 0)min_x = 0;
                    }
                    if(clusters[k].at(j).pt.y < min_y){
                        min_y = clusters[k].at(j).pt.y - expander;
                        if(min_y < 0)min_y = 0;
                    }
                    if(clusters[k].at(j).pt.x > max_x){
                        max_x = clusters[k].at(j).pt.x + expander;
                        if(max_x > img_msg->value.cols)max_x = img_msg->value.cols;
                    }
                    if(clusters[k].at(j).pt.y > max_y){
                        max_y = clusters[k].at(j).pt.y + expander;
                        if(max_y > img_msg->value.rows)max_y = img_msg->value.rows;
                    }
                }
                int size_x= max_x-min_x;
                if(size_x<0){size_x=1; min_x=1;}
                int size_y= max_y-min_y;
                if(size_y<0){size_y=1; min_y=1;}
                cv::Rect roi = cv::Rect(min_x,min_y, size_x , size_y);
                cv::Mat roiImg = cv::Mat(size_x,size_y, out->value.type());
                roiImg = img_msg->value(roi);
                roivectors.push_back(roiImg);
                cv::drawKeypoints(out->value, clusters[k], out->value, random);
                cv::rectangle(out->value, roi,random,2);
            }
        }
        out->value = roivectors[mostcluster];
        out_->publish(out);
    }
}

void KeypointCluster::updateroi(int slot){

    QtHelper::clearLayout(opt->layout());
    QBoxLayout* layout = dynamic_cast<QBoxLayout*> (opt->layout());
    assert(layout);

    img_roi = QtHelper::makeSlider(layout, "roi", 1, 1, numberofrois_-1);
    QObject::connect(img_roi, SIGNAL(valueChanged(int)), this, SLOT(updateroislot(int)));
}

std::vector<std::vector<cv::KeyPoint> > KeypointCluster::DBSCAN_keypoints(std::vector<cv::KeyPoint> keypoints, float eps, int minPts)
{
std::vector<std::vector <cv::KeyPoint> > clusters;
std::vector<bool> clustered;
std::vector<int> noise;
std::vector<bool> visited;
std::vector<int> neighborPts;
std::vector<int> neighborPts_;
int c;

    for(int i = 0; i< keypoints.size(); i++){
        visited.push_back(false);
        clustered.push_back(false);
    }

    c=0;
    clusters.push_back(std::vector<cv::KeyPoint>());

    for(int i = 0; i< keypoints.size(); i++){
        if(!visited[i]){

        visited[i] = true;
        neighborPts = regionQuery(keypoints,keypoints.at(i),eps);

        if(neighborPts.size() < minPts){
            noise.push_back(i); //dont know if this is right
        }else{
            clusters.push_back(std::vector<cv::KeyPoint>());
            c++;

            //expand cluster
            // add P to cluster c

            clusters[c].push_back(keypoints.at(i));

            //for each point P' in neighborPts
            for(int j = 0; j < neighborPts.size(); j++)
                {
                    //if P' is not visited
                    if(!visited[neighborPts[j]])
                    {
                        //Mark P' as visited
                        visited[neighborPts[j]] = true;
                        neighborPts_ = regionQuery(keypoints,keypoints.at(neighborPts[j]),eps);
                        if(neighborPts_.size() >= minPts)
                        {
                            neighborPts.insert(neighborPts.end(),neighborPts_.begin(),neighborPts_.end());
                        }
                    }
                    // if P' is not yet a member of any cluster
                    // add P' to cluster c
                    if(!clustered[neighborPts[j]])
                        clusters[c].push_back(keypoints.at(neighborPts[j]));
                }
            }
        }
    }
return clusters;
}

std::vector<int> KeypointCluster::regionQuery(std::vector<cv::KeyPoint> keypoints, cv::KeyPoint &keypoint, float eps){
    float dist;
    eps = eps*eps;
    std::vector<int> retKeys;
    for(int i = 0; i< keypoints.size(); i++)
    {
        dist = pow((keypoint.pt.x - keypoints.at(i).pt.x),2)+pow((keypoint.pt.y - keypoints.at(i).pt.y),2);
        if(dist <= eps && dist != 0.0f)
        {
            retKeys.push_back(i);
        }
    }
    return retKeys;
}



