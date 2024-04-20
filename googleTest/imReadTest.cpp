#include<gtest/gtest.h>
#include<iostream>
#include<opencv4/opencv2/opencv.hpp>
#include<string>
#include"calibration.h"
using namespace CQGCalibration;
TEST(calibrationCamera, imTest){
    auto path = std::string("/home/cqg/githubProject/calibration/data/L");
    calConfig cameraConfig;
    cameraConfig.block_size = 10;
    cameraConfig.height = 2048;
    cameraConfig.width = 2448;
    cameraConfig.N = 13;
    cameraConfig.points_per_col = 8;
    cameraConfig.points_per_row = 11;
    cameraCalibration camera;
    if(camera.initation(cameraConfig)){
        camera.imRead(path);
        camera.runCalibration();
        camera.evaluator();
    }
}