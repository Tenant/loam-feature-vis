#include "dsvlprocessor.h"
#include <unordered_set>


DsvlProcessor::DsvlProcessor(std::string dsvl_, std::string calib_):
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

    loam::LaserOdometryParams laserOdometryParams = loam::LaserOdometryParams();
    laserOdometry = loam::LaserOdometry(laserOdometryParams);

    loam::LaserMappingParams laserMappingParams= loam::LaserMappingParams();
    laserMapping= loam::LaserMapping(laserMappingParams);

    viewer->setBackgroundColor(0,0,0);
    viewer->addCoordinateSystem(1.0);

    loadCalibFile(calib_);

    dsvlbytesiz = sizeof (ONEDSVDATA);
    dsvbytesiz = dsvlbytesiz-sizeof(int)*LINES_PER_BLK*PNTS_PER_LINE;
    dFrmNum=0;

    dfp.open(dsvl_.c_str(), std::ios_base::binary);
    if (!dfp.is_open()){
        printf("File open failure : %s\n", dsvl_.c_str());
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
        if (num%100==0) {
            printf("%d (%d)\n",num,dFrmNum);
        }

        if (num == 100) break;
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

    transformPclToIMU();

    loam::Twist transform;
    transform.setZero();

    laserOdometry.spin(cornerPointsSharp.makeShared(),
                       cornerPointsLessSharp.makeShared(),
                       surfacePointsFlat.makeShared(),
                       surfacePointsLessFlat.makeShared(),
                       laserCloud.makeShared(),
                       transform, millsec);
    transformSum = laserOdometry.transformSum();

    laserMapping.spin(cornerPointsSharp.makeShared(),
                      surfacePointsFlat.makeShared(),
                      laserCloud.makeShared(),
                      transformSum, millsec);
    transformSum = laserMapping.transformSum();
}

void DsvlProcessor::printLog() {
    std::printf("[timestamp, %d], ", onefrm->dsv[0].millisec);
    std::printf("[laserCloud, %d], [cornerPointsSharp, %d], [cornerPointsLessSharp, %d], [surfacePointsFlat, %d], [surfacePointsLessFlat, %d]\n",\
    laserCloud.size(), cornerPointsSharp.size(), cornerPointsLessSharp.size(),\
    surfacePointsFlat.size(), surfacePointsLessFlat.size());
    std::printf("[x, %f], [y, %f], [z, %f], [pitch, %f], [yaw, %f], [roll, %f]\n",
            transformSum.pos.x(),
            transformSum.pos.y(),
            transformSum.pos.z(),
            transformSum.rot_x.deg(),
            transformSum.rot_y.deg(),
            transformSum.rot_z.deg());
}

void DsvlProcessor::transformToIMU(const pcl::PointXYZI &pi, pcl::PointXYZI &po) {
    po = pi;
    cv::Point3d pt_ = _cTransMat * cv::Point3d(pi.z, pi.x, pi.y) + cv::Point3d(calib_shv.x, calib_shv.y, calib_shv.z);
    po.x = pt_.y; po.y = pt_.z; po.z = pt_.x;
}

void DsvlProcessor::loadCalibFile(string filename) {
    FILE* fCalib = std::fopen(filename.c_str(), "r");
    fscanf(fCalib, "rot %lf %lf %lf\n", &calib_ang.y, &calib_ang.x, &calib_ang.z);
    fscanf(fCalib, "shv %lf %lf %lf\n", &calib_shv.x, &calib_shv.y, &calib_shv.z);
    calib_ang.x *= M_PI/180.0;
    calib_ang.y *= M_PI/180.0;
    calib_ang.z *= M_PI/180.0;
    std::fclose(fCalib);

    cv::Mat crot_x = (cv::Mat_<double>(4,4)<<1,0,0,0,0,cos(calib_ang.x),-sin(calib_ang.x),0,0,sin(calib_ang.x),cos(calib_ang.x),0,0,0,0,1);
    cv::Mat crot_y = (cv::Mat_<double>(4,4)<<cos(calib_ang.y),0,sin(calib_ang.y),0,0,1,0,0,-sin(calib_ang.y),0,cos(calib_ang.y),0,0,0,0,1);
    cv::Mat crot_z = (cv::Mat_<double>(4,4)<<cos(calib_ang.z),-sin(calib_ang.z),0,0,sin(calib_ang.z),cos(calib_ang.z),0,0,0,0,1,0,0,0,0,1);
    cv::Mat cshv = (cv::Mat_<double>(4,4)<<0,0,0,calib_shv.x,0,0,0,calib_shv.y,0,0,0,calib_shv.z,0,0,0,0);
    cv::Mat crot = crot_z*crot_y*crot_x + cshv;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<4;j++) {
            if (j < 3) {
                _cTransMat(i, j) = crot.at<double>(i,j);
            }
        }
    }
}

void DsvlProcessor::transformPclToIMU() {
    size_t laserCloudNum = laserCloud.points.size();
    for (int i = 0; i < laserCloudNum; i++) {
        transformToIMU(laserCloud.points[i], laserCloud.points[i]);
    }

    size_t cornerPointsSharpNum = cornerPointsSharp.points.size();
    for (int i = 0; i < cornerPointsSharpNum; i++) {
        transformToIMU(cornerPointsSharp.points[i], cornerPointsSharp.points[i]);
    }

    size_t cornerPointsLessSharpNum = cornerPointsLessSharp.points.size();
    for (int i = 0; i < cornerPointsLessSharpNum; i++) {
        transformToIMU(cornerPointsLessSharp.points[i], cornerPointsLessSharp.points[i]);
    }

    size_t surfacePointsFlatNum = surfacePointsFlat.points.size();
    for (int i = 0; i < surfacePointsFlatNum; i++) {
        transformToIMU(surfacePointsFlat.points[i], surfacePointsFlat.points[i]);
    }

    size_t surfacePointsLessFlatNum = surfacePointsFlat.points.size();
    for (int i = 0; i < surfacePointsLessFlatNum; i++) {
        transformToIMU(surfacePointsLessFlat.points[i], surfacePointsLessFlat.points[i]);
    }
}

