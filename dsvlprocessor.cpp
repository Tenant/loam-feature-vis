#include "dsvlprocessor.h"
#include <unordered_set>


DsvlProcessor::DsvlProcessor(string filename):
viewer(new pcl::visualization::PCLVisualizer("Feature-Vis")),
pts(new pcl::PointCloud<pcl::PointXYZI>()),
handler(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>(pts, "z"))
{
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
        if(first_frame)
        {
            viewer->addPointCloud<pcl::PointXYZI>(pts, *handler, "DYP");
            first_frame = false;
        }
        else {
            viewer->updatePointCloud<pcl::PointXYZI>(pts, *handler, "DYP");
        }
        viewer->spinOnce(100);
        std::printf("%d\n", onefrm->dsv[0].millisec);
    }
}

void DsvlProcessor::ProcessOneFrame() {
    point3fi *p;
    pts->clear();
    for (int i=0; i<BKNUM_PER_FRM; i++) {
        for (int j = 0; j < LINES_PER_BLK; j++) {
            for (int k = 0; k < PNTS_PER_LINE; k++) {
                p = &onefrm->dsv[i].points[j * PNTS_PER_LINE + k];
                if (!p->i)
                    continue;
                pcl::PointXYZI p_;
                p_.x = p->x; p_.y = p->y; p_.z = p->z; p_.intensity = p->i;
                pts->push_back(p_);
            }
        }
    }
}

