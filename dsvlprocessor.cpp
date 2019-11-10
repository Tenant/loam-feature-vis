#include "dsvlprocessor.h"
#include <unordered_set>


DsvlProcessor::DsvlProcessor(string filename):
viewer(new pcl::visualization::PCLVisualizer("Feature-Vis")),
pts(new pcl::PointCloud<pointT>()),
laserCloud(),
cornerPointsSharp(),
cornerPointsLessSharp(),
surfacePointsFlat(),
surfacePointsLessFlat(),
handler(new pcl::visualization::PointCloudColorHandlerGenericField<pointT>(pts, "z"))
{
    loam::MultiScanMapper scanMapper = loam::MultiScanMapper(-16,7,40);
    loam::ScanRegistrationParams params = loam::ScanRegistrationParams(0.1,200,6,5,2,4,0.2,0.1,200,"none");
    featureExtractor = loam::MultiScanRegistration(scanMapper, params);

    viewer->setBackgroundColor(0,0,0);
    viewer->addCoordinateSystem(1.0);

    dsvlbytesiz = sizeof (ONEDSVDATA);
    dsvbytesiz = dsvlbytesiz-sizeof(int)*LINES_PER_BLK*PNTS_PER_LINE;
    dFrmNum=0;

    dfp.open(filename.c_str(), std::ios_base::binary);
    if (!dfp.is_open()){
        printf("File open failure : %s\n", filename.c_str());
        isRunning = false;
    }
    else{
        isRunning = true;
    }
}


bool DsvlProcessor::ReadOneDsvlFrame()
{
    int		i;
    for (i=0; i<BKNUM_PER_FRM; i++) {
        dfp.read((char *)&onefrm->dsv[i], dsvlbytesiz);
        if (dfp.gcount() != dsvlbytesiz)
            break;
    }

    if (i<BKNUM_PER_FRM)
        return false;
    else
        return true;
}


void DsvlProcessor::Processing()
{
    dfp.seekg(0, std::ios_base::end);
    dFrmNum = dfp.tellg() / 180 / dsvlbytesiz;

    dfp.seekg(0, std::ios_base::beg);

    onefrm= new ONEDSVFRAME[1];

    int num = 0;
    bool first_frame = true;

    while (ReadOneDsvlFrame () && isRunning)
    {
        if (num%100==0) printf("%d (%d)\n",num,dFrmNum);
        num++;

        ProcessOneFrame();

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colorLaserCloud(laserCloud.makeShared(), 255, 255, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colorCornerPointsSharp(cornerPointsSharp.makeShared(), 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colorCornerPointsLessSharp(cornerPointsLessSharp.makeShared(), 255, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colorSurfacePointsFlat(surfacePointsFlat.makeShared(), 0, 255, 0);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cornerPointsSharp");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cornerPointsLessSharp");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "surfacePointsFlat");
        if(first_frame)
        {
//            viewer->addPointCloud<pointT>(pts, *handler, "DYP");
            viewer->addPointCloud<pcl::PointXYZI>(laserCloud.makeShared(), colorLaserCloud, "laserCloud");
            viewer->addPointCloud<pcl::PointXYZI>(cornerPointsSharp.makeShared(), colorCornerPointsSharp, "cornerPointsSharp");
            viewer->addPointCloud<pcl::PointXYZI>(cornerPointsLessSharp.makeShared(), colorCornerPointsLessSharp, "cornerPointsLessSharp");
            viewer->addPointCloud<pcl::PointXYZI>(surfacePointsFlat.makeShared(), colorSurfacePointsFlat, "surfacePointsFlat");
            first_frame = false;
        }
        else {
//            viewer->updatePointCloud<pointT>(pts, *handler, "DYP");
            viewer->updatePointCloud<pcl::PointXYZI>(laserCloud.makeShared(), colorLaserCloud, "laserCloud");
            viewer->updatePointCloud<pcl::PointXYZI>(cornerPointsSharp.makeShared(), colorCornerPointsSharp, "cornerPointsSharp");
            viewer->updatePointCloud<pcl::PointXYZI>(cornerPointsLessSharp.makeShared(), colorCornerPointsLessSharp, "cornerPointsLessSharp");
            viewer->updatePointCloud<pcl::PointXYZI>(surfacePointsFlat.makeShared(), colorSurfacePointsFlat, "surfacePointsFlat");
        }
        viewer->spinOnce(100);

        printLog();
    }
}

void DsvlProcessor::ProcessOneFrame() {
    loam::Time millsec = onefrm->dsv[0].millisec;

    point3fi *p;
    pts->clear();
    for (int i=0; i<BKNUM_PER_FRM; i++) {
        for (int j = 0; j < LINES_PER_BLK; j++) {
            for (int k = 0; k < PNTS_PER_LINE; k++) {
                p = &onefrm->dsv[i].points[j * PNTS_PER_LINE + k];
                if (!p->i)
                    continue;
                pointT p_;
                p_.x = p->x; p_.y = p->y; p_.z = p->z;
                pts->push_back(p_);
            }
        }
    }

    featureExtractor.process(*pts, millsec);

    laserCloud = featureExtractor.laserCloud();
    cornerPointsSharp = featureExtractor.cornerPointsSharp();
    cornerPointsLessSharp = featureExtractor.cornerPointsLessSharp();
    surfacePointsFlat = featureExtractor.surfacePointsFlat();
    surfacePointsLessFlat = featureExtractor.surfacePointsLessFlat();
}

void DsvlProcessor::printLog() {
    std::printf("[timestamp, %d], ", onefrm->dsv[0].millisec);
    std::printf("[laserCloud, %d], [cornerPointsSharp, %d], [cornerPointsLessSharp, %d], [surfacePointsFlat, %d], [surfacePointsLessFlat, %d]\n",\
    laserCloud.size(), cornerPointsSharp.size(), cornerPointsLessSharp.size(),\
    surfacePointsFlat.size(), surfacePointsLessFlat.size());
}

