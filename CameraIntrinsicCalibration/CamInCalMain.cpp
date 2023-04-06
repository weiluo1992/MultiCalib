#include <iostream>
#include <dirent.h>

#include "CameraIntrinsicCalibration.h"
#include <opencv2/opencv.hpp>

const char usage[] =
"\n \tCamInCalMain <calibration_image_dir> \n"
"example:\n\t"
"CamInCalMain ./data/\n";

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << argv[0] << usage;
        return 1;
    }

    std::string input_image_path = argv[1];
    // cv::Mat image = cv::imread(input_image_path.c_str(), 0);

    CameraIntrinsicCalibration calibrator;
    calibrator.Calibrate(input_image_path);
    return 0;
}