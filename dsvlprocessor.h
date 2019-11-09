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
//
#include "types.h"

using namespace std;
using namespace cv;


class DsvlProcessor
{
public:
    DsvlProcessor(std::string filename);
    bool ReadOneDsvlFrame ();
    void Processing();
    void ProcessOneFrame ();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts;
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>::Ptr handler;

    int dsvlbytesiz;
    int dsvbytesiz;
    int labbytesiz;
    int dFrmNum;
    ONEDSVFRAME	*onefrm;
    std::ifstream dfp;
    bool isRunning;
};

#endif // DSVLPROCESSOR_H
