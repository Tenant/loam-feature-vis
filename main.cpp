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
//ofstream fout;


int HEIGHT;
int WIDTH;
int VIS_HEIGHT;
int VIS_WIDTH;
double FAC_HEIGHT;


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

    DsvlProcessor dsvl(dsvlfilename, calibFileName);
    dsvl.Processing();

    cout << "Over!" << endl;
//    fout.close();
    return 0;
}
