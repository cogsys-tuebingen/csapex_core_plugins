#ifndef OBJECT_HOMOGRAPHY_H
#define OBJECT_HOMOGRAPHY_H

#endif // OBJECT_HOMOGRAPHY_H
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <QtGui>

//#include <opencv2/opencv.hpp>


class Surfhomography{

public:

Surfhomography();
void readme();
/** @function main */
cv::Mat calculation(const cv::Mat img_object, const cv::Mat img_scene, std::vector<cv::KeyPoint> &keypoints_object, std::vector<cv::KeyPoint> &keypoints_scene, cv::Mat &descriptor_object, cv::Mat &descriptor_scene);

};
