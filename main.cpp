#include <iostream>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include "dsvlprocessor.h"

using namespace std;

std::string imgname;
std::string videoName;
std::string videoNameSeg;
std::string bin_path;
std::string label_path;
std::string poseFileName;
std::string outputDir;
std::string tag_path;
ofstream fout;


int HEIGHT;
int WIDTH;
int VIS_HEIGHT;
int VIS_WIDTH;
double FAC_HEIGHT;

point3d calib_shv, calib_ang;
bool flagBbox;

void loadCalibFile(std::string calibFileName)
{
    FILE* fCalib = std::fopen(calibFileName.c_str(), "r");
    fscanf(fCalib, "rot %lf %lf %lf\n", &calib_ang.y, &calib_ang.x, &calib_ang.z);
    fscanf(fCalib, "shv %lf %lf %lf\n", &calib_shv.x, &calib_shv.y, &calib_shv.z);
    calib_ang.x *= M_PI/180.0;
    calib_ang.y *= M_PI/180.0;
    calib_ang.z *= M_PI/180.0;
    std::fclose(fCalib);
}

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::fprintf(stderr, "Args not enough !\n");
        std::printf("[Usage] ./GenerateSamplesForPointLabeler [dsvl] [calib]\n");
        std::printf("For example:\n");
        std::printf("[dsvl](required): 20190331133302_4-seg.dsvl\n");
        std::printf("[calib](required): P40n.calib\n");
        return 0;
    }
    std::string dsvlfilename(argv[1]);

    // 激光雷达到GPS标定文件
    std::string calibFileName(argv[2]);

    // 32线激光雷达，投影到距离图像的尺寸
    HEIGHT = 32;
    WIDTH  = 1080;
    // 用于可视化的尺寸
    VIS_HEIGHT = 144;
    VIS_WIDTH  = 1080;
    FAC_HEIGHT = (double)VIS_HEIGHT / (double)HEIGHT;

    loadCalibFile(calibFileName);

    DsvlProcessor dsvl(dsvlfilename);
    dsvl.Processing();

    cout << "Over!" << endl;
    fout.close();
    return 0;
}
