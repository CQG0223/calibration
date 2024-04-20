#include"calibration.h"
using namespace CQGCalibration;

bool cameraCalibration::initation(const CQGCalibration::calConfig& config){
    if(config.N > 0 && config.height > 0 && config.width > 0){
        config_ = config;
    }else{
        return false;
    }
    corner_size = cv::Size(config_.points_per_row, config_.points_per_col);
    image_size = cv::Size(config_.width, config_.height);
    block_size = cv::Size(config_.block_size, config_.block_size);
    return true;
}

void cameraCalibration::imRead(const std::string path){
    fs::path dir_path = path;
    auto bmpFiles = getBmpFiles(dir_path);
    cv::Mat image;
    for(const auto& file : bmpFiles){
        image = cv::imread(file,0);
        cBoardcorimage_.push_back(image);
    }
}

bool cameraCalibration::runCalibration(){
    if(cBoardcorimage_.empty()) return false;
    const float block_size_mm = config_.block_size;    //每个小方格实际大小(mm)
    int image_nums = 0;                          //有效图片数量统计

    for(auto images : cBoardcorimage_){
        bool success = cv::findChessboardCorners(images,corner_size,points_per_image);
        if(!success){
            throw std::runtime_error("Do not find corners");
        }else{
            cv::find4QuadCornerSubpix(images,points_per_image,cv::Size(5,5));   //亚像素角点
            points_all_images.push_back(points_per_image);
        }
        image_nums++;
    }
    if(image_nums != config_.N) return false;
    cv::Point3f point3D;
    for(int i = 0; i < corner_size.height; i++){
        for(int j = 0; j < corner_size.width; j++){
            point3D = cv::Point3f(block_size.width * j, block_size.height * i, 0);
            points3D_per_image.push_back(point3D);
        }
    }
    points3D_all_images = std::vector<std::vector<cv::Point3f>>(image_nums, points3D_per_image); //保存所有图像角点的三维坐标
    cv::calibrateCamera(points3D_all_images, points_all_images, 
                        image_size, singleCameraConfig_.cameraMat, 
                        singleCameraConfig_.distCoeffs, 
                        singleCameraConfig_.rotationMat, 
                        singleCameraConfig_.translationMat, 0);
    return true;
}

bool cameraCalibration::evaluator(){
    const auto image_size = config_.N;
    double total_err = 0.0;         //所有图像平均误差总和
    double err = 0.0;               //每幅图像平均误差
    int point_counts = corner_size.area();
    std::vector<cv::Point2f> points_reproject;  //重投影点
    for(int i = 0; i < image_size; i++){
        points_per_image = points_all_images[i];
        points3D_per_image = points3D_all_images[i];
        cv::projectPoints(points3D_per_image, 
                    singleCameraConfig_.rotationMat[i],
                    singleCameraConfig_.translationMat[i],
                    singleCameraConfig_.cameraMat,
                    singleCameraConfig_.distCoeffs,points_reproject);
        cv::Mat detect_points_Mat(1, points_per_image.size(), CV_32FC2);  // 变为1*S的矩阵,2通道保存提取角点的像素坐标
        cv::Mat points_reproj_Mat(1, points_reproject.size(), CV_32FC2);  // 变为1*S的矩阵,2通道保存投影角点的像素坐标
        for (int j = 0; j < points_per_image.size(); j++){
			detect_points_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(points_per_image[j].x, points_per_image[j].y);
			points_reproj_Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(points_reproject[j].x, points_reproject[j].y);
		}
        err = norm(points_reproj_Mat, detect_points_Mat, cv::NormTypes::NORM_L2);  // 计算两者之间的误差
        total_err += err /= point_counts;
        std::cout<<"The average error of "<<i+1<<" : "<<err<<"pixels"<<std::endl;;
    }
    std::cout<<"The average error of images "<< total_err / image_size << "pixels"<<std::endl;
    return true;
}

std::vector<fs::path> cameraCalibration::getBmpFiles(const fs::path& dirPath){
    std::vector<fs::path> bmp_files;
    for (const auto& entry : fs::directory_iterator(dirPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".bmp") {
            bmp_files.push_back(entry.path());
        }
    }
    return bmp_files;
}