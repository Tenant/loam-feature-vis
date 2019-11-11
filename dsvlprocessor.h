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

using namespace std;

typedef pcl::PointXYZ pointT;

class DsvlProcessor
{
public:
    DsvlProcessor(std::string filename);
    bool ReadOneDsvlFrame ();
    void Processing();
    void ProcessOneFrame ();
    void printLog();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pointT>::Ptr pts;
    pcl::visualization::PointCloudColorHandlerGenericField<pointT>::Ptr handler;

    pcl::PointCloud<pcl::PointXYZI> laserCloud;
    pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp;
    pcl::PointCloud<pcl::PointXYZI> cornerPointsLessSharp;
    pcl::PointCloud<pcl::PointXYZI> surfacePointsFlat;
    pcl::PointCloud<pcl::PointXYZI> surfacePointsLessFlat;

    int dsvlbytesiz;
    int dsvbytesiz;
    int labbytesiz;
    int dFrmNum;
    ONEDSVFRAME	*onefrm;
    std::ifstream dfp;
    bool isRunning;

    loam::MultiScanRegistration featureExtractor;
    loam::LaserOdometry laserOdometry;
};

#endif // DSVLPROCESSOR_H
