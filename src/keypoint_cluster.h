#ifndef KEYPOINT_CLUSTER_H
#define KEYPOINT_CLUSTER_H

#include <csapex/model/boxed_object.h>
#include <csapex/csapex_fwd.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>

/// SYSTEM
#include <QSlider>
#include <QFrame>


namespace csapex
{

class KeypointCluster : public BoxedObject
{
    Q_OBJECT

public:
    KeypointCluster();

    /// insert GUI elements
    virtual void fill(QBoxLayout* layout);

private Q_SLOTS:
    /// callback when a message arrives
    void messageArrived(ConnectorIn* source);
    void updateeps(int value);
    void updateminpts(int value);
    void updateroislot(int value);

private:
    /// connectors that were added to the parent box
    ConnectorIn* in_keypoints_;
    ConnectorIn* in_image_;

    ConnectorOut* out_;

    QSlider *epsilon_;
    QSlider *minpoints_;
    QSlider *img_roi;
    QFrame  *opt;

    int numberofrois_;
    int chosenroi_;
    void updateroi(int slot);

    /// flags to remember, whether both images have been received
    bool has_image_;
    bool has_keypoint_;

    /// flag to remember, which image to re-publish
    bool publish_a_;

private:
    std::vector<std::vector<cv::KeyPoint> > DBSCAN_keypoints(std::vector<cv::KeyPoint> keypoints, float eps, int minPts);
    std::vector<int> regionQuery(std::vector<cv::KeyPoint> keypoints, cv::KeyPoint &keypoint, float eps);
    void expandCluster(std::vector<cv::KeyPoint> P, std::vector<cv::KeyPoint> *neighborPts,int c, float eps);

    float eps_;
    int minPts_;

};

}

#endif // KEYPOINT_CLUSTER_H
