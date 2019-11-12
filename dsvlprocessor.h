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
    bool ReadOneDsvlFrame ();
    void Processing();
    void ProcessOneFrame ();
    void printLog();
    void loadCalibFile(std::string);
    void transformToIMU(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
    void transformPclToIMU();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pointT>::Ptr pts;
    pcl::visualization::PointCloudColorHandlerGenericField<pointT>::Ptr handler;

    pcl::PointCloud<pcl::PointXYZI> laserCloud;
    pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp;
    pcl::PointCloud<pcl::PointXYZI> cornerPointsLessSharp;
    pcl::PointCloud<pcl::PointXYZI> surfacePointsFlat;
    pcl::PointCloud<pcl::PointXYZI> surfacePointsLessFlat;

    loam::Twist transformSum;

    int dsvlbytesiz;
    int dsvbytesiz;
    int labbytesiz;
    int dFrmNum;
    ONEDSVFRAME	*onefrm;
    std::ifstream dfp;
    bool isRunning;

    point3d	_ang;
    point3d	_shv;
    point3d _shv0;
    point3d calib_ang;
    point3d calib_shv;
    cv::Matx33d _cTransMat;
    cv::Matx33d _rTransMat;

    loam::MultiScanRegistration featureExtractor;
    loam::LaserOdometry laserOdometry;
    loam::LaserMapping laserMapping;
};

#endif // DSVLPROCESSOR_H
