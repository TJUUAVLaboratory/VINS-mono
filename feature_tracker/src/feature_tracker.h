#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"
/*
**********************************************
*
*
**********************************************
*/
using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);



class FeatureTracker
{
  public:
    FeatureTracker(); //构造函数没有内容

    void readImage(const cv::Mat &_img,double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    // 鱼眼相机需要去除边缘一圈噪声的mask 图像
    cv::Mat mask;
    cv::Mat fisheye_mask;
    //上一帧图像，当前帧图像，和新进的需要光流追踪的图像
    cv::Mat prev_img, cur_img, forw_img;

    vector<cv::Point2f> n_pts; //特征提取，提取到的角点
    //前一帧的特征点，当前帧的特征点，以及需要光流追踪计算的特征点
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;

    //去畸变的点
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity; //计算点的速度
    vector<int> ids;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    static int n_id;
};
