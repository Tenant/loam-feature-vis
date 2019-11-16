#include "dsvlprocessor.h"
#include <unordered_set>


DsvlProcessor::DsvlProcessor(std::string dsvl_, std::string calib_):
isInited(false),
num(0),
_canvas(600,600,CV_8UC3, cv::Scalar::all(1)),
viewer(new pcl::visualization::PCLVisualizer("Feature-Vis")),
pts(new pcl::PointCloud<pointT>()),
laserCloud(),
cornerPointsSharp(),
cornerPointsLessSharp(),
surfacePointsFlat(),
surfacePointsLessFlat(),
_map(new pcl::PointCloud<pcl::PointXYZI>()),
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

    fout = std::fopen("traj.nav", "w");

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

    while (ReadOneDsvlFrame () && isRunning)
    {
        if (num%100==0) {
            printf("%d (%d)\n",num,dFrmNum);
        }
        num++;
        if (num < 300) continue;
        if (num > 450) break;

        ProcessOneFrame();

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colorLaserCloud(laserCloud.makeShared(), 255, 255, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colorCornerPointsSharp(cornerPointsSharp.makeShared(), 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colorCornerPointsLessSharp(cornerPointsLessSharp.makeShared(), 255, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colorSurfacePointsFlat(surfacePointsFlat.makeShared(), 0, 255, 0);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cornerPointsSharp");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cornerPointsLessSharp");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "surfacePointsFlat");
        if(!isInited)
        {
            viewer->addPointCloud<pcl::PointXYZI>(laserCloud.makeShared(), colorLaserCloud, "laserCloud");
            viewer->addPointCloud<pcl::PointXYZI>(cornerPointsSharp.makeShared(), colorCornerPointsSharp, "cornerPointsSharp");
            viewer->addPointCloud<pcl::PointXYZI>(cornerPointsLessSharp.makeShared(), colorCornerPointsLessSharp, "cornerPointsLessSharp");
            viewer->addPointCloud<pcl::PointXYZI>(surfacePointsFlat.makeShared(), colorSurfacePointsFlat, "surfacePointsFlat");
            isInited = true;
        }
        else {
            viewer->updatePointCloud<pcl::PointXYZI>(laserCloud.makeShared(), colorLaserCloud, "laserCloud");
            viewer->updatePointCloud<pcl::PointXYZI>(cornerPointsSharp.makeShared(), colorCornerPointsSharp, "cornerPointsSharp");
            viewer->updatePointCloud<pcl::PointXYZI>(cornerPointsLessSharp.makeShared(), colorCornerPointsLessSharp, "cornerPointsLessSharp");
            viewer->updatePointCloud<pcl::PointXYZI>(surfacePointsFlat.makeShared(), colorSurfacePointsFlat, "surfacePointsFlat");
        }
        viewer->spinOnce(100);

//        int ix0, ix1, iy0, iy1;
//        bool bound0 = transformToCanvas(_shv0.x, _shv0.z, ix0, iy0);
//        bool bound1 = transformToCanvas(_shv.x, _shv.z, ix1, iy1);
//        if (bound0 && bound1)
//            cv::line(_canvas, cv::Point(ix0,iy0), cv::Point(ix1,iy1), cv::Scalar(0,0,255), 5, CV_AA);
//        cv::imshow("Traj", _canvas);
//        cv::waitKey(100);

        printLog();
//        savePointCloud();
    }

    pcl::PCDWriter pclWriter;
//    pcl::VoxelGrid<pcl::PointXYZI> mapFilter;
//    mapFilter.setLeafSize(0.1,0.1,0.1);
//    mapFilter.setInputCloud(_map.makeShared());
//    mapFilter.filter(_map);
    pclWriter.write("map-slam-fined-1.pcd",*_map);
}

void DsvlProcessor::ProcessOneFrame() {
    loam::Time millsec = onefrm->dsv[0].millisec;
    _ang = onefrm->dsv[0].ang;
    _shv = onefrm->dsv[0].shv;

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

    updateTransformToInit();

    featureExtractor.process(*pts, millsec);
    laserCloud = featureExtractor.laserCloud();
    cornerPointsSharp = featureExtractor.cornerPointsSharp();
    cornerPointsLessSharp = featureExtractor.cornerPointsLessSharp();
    surfacePointsFlat = featureExtractor.surfacePointsFlat();
    surfacePointsLessFlat = featureExtractor.surfacePointsLessFlat();

    transformPclToIMU();

    if(!isInited) {
        _ang0.x = 0; _ang0.y = 0; _ang.z = 0;
        _initAng = _ang0;
        _shv0 = _shv;
        _initShv = _shv;
    }

    loam::Twist imuTrans;
    imuTrans.pos = loam::Vector3(_shv.y - _shv0.y, _shv.z - _shv0.z, _shv.x - _shv0.x);
    imuTrans.rot_x = loam::Angle(_ang.y - _ang0.y);
    imuTrans.rot_y = loam::Angle(_ang.z - _ang0.z);
    imuTrans.rot_z = loam::Angle(_ang.x - _ang0.x);

    laserOdometry.spin(cornerPointsSharp.makeShared(),
                       cornerPointsLessSharp.makeShared(),
                       surfacePointsFlat.makeShared(),
                       surfacePointsLessFlat.makeShared(),
                       laserCloud.makeShared(),
                       imuTrans, millsec);
    _transformSum = laserOdometry.transformSum();

    laserMapping.spin(cornerPointsSharp.makeShared(),
                      surfacePointsFlat.makeShared(),
                      laserCloud.makeShared(),
                      _transformSum, millsec);
    _transformAftMapped = laserMapping.transformAftMapped();
    laserCloudOri = laserMapping.laserCloudOri();
    coeffSel = laserMapping.coeffSel();
    laserMapping.generateMapCloud(_map);

    transformImuToInit();

//    _map += laserCloud;

    _ang0 = _ang;
    _shv0 = _shv;
}

void DsvlProcessor::printLog() {
    std::printf("[num, %d], [timestamp, %d], ", num, onefrm->dsv[0].millisec);
    std::printf("[laserCloud, %zu], [cornerPointsSharp, %zu], [cornerPointsLessSharp, %zu], [surfacePointsFlat, %zu], [surfacePointsLessFlat, %zu]\n",\
    laserCloud.size(), cornerPointsSharp.size(), cornerPointsLessSharp.size(),\
    surfacePointsFlat.size(), surfacePointsLessFlat.size());
    std::printf("[x, %f], [y, %f], [z, %f], [pitch, %f], [yaw, %f], [roll, %f]\n",
            _transformSum.pos.x(),
            _transformSum.pos.y(),
            _transformSum.pos.z(),
            _transformSum.rot_x.deg(),
            _transformSum.rot_y.deg(),
            _transformSum.rot_z.deg());
    std::printf("[x, %f], [y, %f], [z, %f], [pitch, %f], [yaw, %f], [roll, %f]\n",
                _transformAftMapped.pos.x(),
                _transformAftMapped.pos.y(),
                _transformAftMapped.pos.z(),
                _transformAftMapped.rot_x.deg(),
                _transformAftMapped.rot_y.deg(),
                _transformAftMapped.rot_z.deg());
    std::printf("[x, %f], [y, %f], [z, %f], [pitch, %f], [yaw, %f], [roll, %f]\n",
                _shv.y - _initShv.y,
                _shv.z - _initShv.z,
                _shv.x - _initShv.x,
                _ang.y * 180 / M_PI,
                _ang.z * 180 / M_PI,
                _ang.x * 180 / M_PI);
    fprintf(fout, "%f, %f, %f, %f, %f, %f\n",
            _transformAftMapped.pos.x(),
            _transformAftMapped.pos.y(),
            _transformAftMapped.pos.z(),
            _transformAftMapped.rot_x.deg(),
            _transformAftMapped.rot_y.deg(),
            _transformAftMapped.rot_z.deg());
}

void DsvlProcessor::updateTransformToInit() {
    point3d shv_now = _shv;
    point3d ang_now = _ang;
    ang_now.x = _ang.y;
    ang_now.y = _ang.x;
    double shv_x = shv_now.x - _initShv.x;
    double shv_y = shv_now.y - _initShv.y;
    double shv_z = shv_now.z - _initShv.z;
    cv::Mat rot_x = (cv::Mat_<double>(4,4)<<1,0,0,0,0,cos(ang_now.x),-sin(ang_now.x),0,0,sin(ang_now.x),cos(ang_now.x),0,0,0,0,1);
    cv::Mat rot_y = (cv::Mat_<double>(4,4)<<cos(ang_now.y),0,sin(ang_now.y),0,0,1,0,0,-sin(ang_now.y),0,cos(ang_now.y),0,0,0,0,1);
    cv::Mat rot_z = (cv::Mat_<double>(4,4)<<cos(ang_now.z),-sin(ang_now.z),0,0,sin(ang_now.z),cos(ang_now.z),0,0,0,0,1,0,0,0,0,1);
    cv::Mat shv = (cv::Mat_<double>(4,4)<<0,0,0,shv_x,0,0,0,shv_y,0,0,0,shv_z,0,0,0,0);
    cv::Mat rot = rot_z*rot_y*rot_x + shv;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<4;j++) {
            if (j < 3) {
                _rTransMat(i, j) = rot.at<double>(i,j);
            }
        }
    }
}

void DsvlProcessor::transformToIMU(const pcl::PointXYZI &pi, pcl::PointXYZI &po) {
    po = pi;
    cv::Point3d pt_ = _cTransMat * cv::Point3d(pi.z, pi.x, pi.y) + cv::Point3d(calib_shv.x, calib_shv.y, calib_shv.z);
    po.x = pt_.y; po.y = pt_.z; po.z = pt_.x;
}

void DsvlProcessor::transformToInit(const pcl::PointXYZI& pi, pcl::PointXYZI& po) {
    po = pi;
    cv::Point3d pt = _rTransMat * cv::Point3d(pi.z, pi.x, pi.y) + cv::Point3d(_shv.x, _shv.y, _shv.z);
    po.x = pt.y; po.y = pt.z; po.z = pt.x;
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

void DsvlProcessor::transformImuToInit() {
    size_t laserCloudNum = laserCloud.points.size();
    for (int i = 0; i < laserCloudNum; i++) {
        transformToInit(laserCloud.points[i], laserCloud.points[i]);
    }

    size_t cornerPointsSharpNum = cornerPointsSharp.points.size();
    for (int i = 0; i < cornerPointsSharpNum; i++) {
        transformToInit(cornerPointsSharp.points[i], cornerPointsSharp.points[i]);
    }

    size_t cornerPointsLessSharpNum = cornerPointsLessSharp.points.size();
    for (int i = 0; i < cornerPointsLessSharpNum; i++) {
        transformToInit(cornerPointsLessSharp.points[i], cornerPointsLessSharp.points[i]);
    }

    size_t surfacePointsFlatNum = surfacePointsFlat.points.size();
    for (int i = 0; i < surfacePointsFlatNum; i++) {
        transformToInit(surfacePointsFlat.points[i], surfacePointsFlat.points[i]);
    }

    size_t surfacePointsLessFlatNum = surfacePointsFlat.points.size();
    for (int i = 0; i < surfacePointsLessFlatNum; i++) {
        transformToInit(surfacePointsLessFlat.points[i], surfacePointsLessFlat.points[i]);
    }

    size_t laserCloudOriNum = laserCloudOri.points.size();
    for (int i = 0; i < laserCloudOriNum; i++) {
        transformToInit(laserCloudOri.points[i], laserCloudOri.points[i]);
    }
}

DsvlProcessor::~DsvlProcessor() {
    std::fclose(fout);
    dfp.close();
}

bool DsvlProcessor::transformToCanvas(const float x_, const float y_, int &ix_, int &iy_) {
    float res = 0.5; // Unit: m
    int cen_x = 300, cen_y = 300;
    ix_ = int(x_ / res) + cen_x;
    iy_ = int(y_ / res) + cen_y;

    if (ix_ >= 0 && ix_ < 600 && iy_ >= 0 && iy_ < 600) {
        return true;
    }
    return false;
}

void DsvlProcessor::savePointCloud() {
    if (!laserCloud.empty() && !laserCloudOri.empty() && !coeffSel.empty()) {
        char name[20];
        pcl::PCDWriter writer;
        sprintf(name, "export/pcl-%d.pcd", num);
        writer.write(name,laserCloud);
        sprintf(name, "export/ori-%d.pcd", num);
        writer.write(name, laserCloudOri);
        sprintf(name, "export/eff-%d.pcd", num);
        writer.write(name, coeffSel);
    }
}
