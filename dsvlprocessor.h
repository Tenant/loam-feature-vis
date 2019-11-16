#ifndef DSVLPROCESSOR_H
#define DSVLPROCESSOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "types.h"
#include "loam_velodyne/MultiScanRegistration.h"
#include "loam_velodyne/LaserOdometry.h"
#include "loam_velodyne/LaserMapping.h"

using namespace std;

typedef pcl::PointXYZ pointT;

class DsvlProcessor
{
public:
    DsvlProcessor(std::string dsvl_, std::string calib_);
    ~DsvlProcessor();
    void Processing();

private:
    bool ReadOneDsvlFrame ();
    void ProcessOneFrame ();
    void printLog();
    void savePointCloud();
    void transformToIMU(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
    void transformToInit(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
    void loadCalibFile(std::string);
    void updateTransformToInit();
    void transformPclToIMU();
    void transformImuToInit();
    bool transformToCanvas(const float x_, const float y_, int& ix_, int& iy_);

private:
    cv::Mat _canvas;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pointT>::Ptr pts;
    pcl::visualization::PointCloudColorHandlerGenericField<pointT>::Ptr handler;

    pcl::PointCloud<pcl::PointXYZI> laserCloud;
    pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp;
    pcl::PointCloud<pcl::PointXYZI> cornerPointsLessSharp;
    pcl::PointCloud<pcl::PointXYZI> surfacePointsFlat;
    pcl::PointCloud<pcl::PointXYZI> surfacePointsLessFlat;
    pcl::PointCloud<pcl::PointXYZI> laserCloudOri;
    pcl::PointCloud<pcl::PointXYZI> coeffSel;

    pcl::PointCloud<pcl::PointXYZI>::Ptr _map;

    loam::Twist _transformSum;
    loam::Twist _transformAftMapped;

    int dsvlbytesiz;
    int dsvbytesiz;
    int labbytesiz;
    int dFrmNum;
    ONEDSVFRAME	*onefrm;
    std::ifstream dfp;
    bool isRunning;
    bool  isInited;
    FILE *fout;
    int num;

    point3d	_ang;
    point3d	_shv;
    point3d _ang0;
    point3d _shv0;
    point3d _initAng;
    point3d _initShv;
    point3d calib_ang;
    point3d calib_shv;
    cv::Matx33d _cTransMat;
    cv::Matx33d _rTransMat;

    loam::MultiScanRegistration featureExtractor;
    loam::LaserOdometry laserOdometry;
    loam::LaserMapping laserMapping;
};

#endif // DSVLPROCESSOR_H
