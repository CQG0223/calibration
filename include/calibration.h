#ifndef CALIBRATION_H_
#define CALIBRATION_H_
#include<opencv4/opencv2/opencv.hpp>
#include<vector>
#include<exception>
#include<iostream>
#include<filesystem>
namespace fs = std::filesystem;
namespace CQGCalibration
{
    struct calConfig
    {
        uint16_t height;
        uint16_t width;
        uint16_t N;
        int points_per_row;
        int points_per_col;
        float block_size;
        calConfig(){
            height = 0;
            width = 0;
            N = 0;
            points_per_col = -1;
            points_per_row = -1;
            block_size = -1;
        }
    };
    struct cameraConfig
    {
        cv::Mat cameraMat;                    // 内参矩阵3*3
        cv::Mat distCoeffs;                   // 畸变矩阵1*5，既考虑径向畸变，又考虑切向
        std::vector<cv::Mat> rotationMat;     // 旋转矩阵
        std::vector<cv::Mat> translationMat;  // 平移矩阵
        cameraConfig(){
            cameraMat = cv::Mat::zeros(cv::Size(3,3),CV_32FC1);
            distCoeffs = cv::Mat::zeros(cv::Size(1,5),CV_32FC1);
        }
    };
    
    class cameraCalibration
    {
    private:
        std::vector<cv::Mat> cBoardcorimage_;    //棋盘矫正图片
        calConfig config_;
        cameraConfig singleCameraConfig_;
        cv::Size corner_size;
        cv::Size image_size;
        std::vector<cv::Point2f> points_per_image;          //缓存每副图检测得到的角点
        std::vector<std::vector<cv::Point2f>> points_all_images;//保存检测得到的所有角点
        cv::Size block_size;
        std::vector<cv::Point3f> points3D_per_image;        //初始化角点三维坐标
        std::vector<std::vector<cv::Point3f>> points3D_all_images;
    public:
        cameraCalibration() : config_{},singleCameraConfig_{}{};
        ~cameraCalibration() = default;
        // \brief 初始化标定程序
        bool initation(const CQGCalibration::calConfig& config);
        // \brief 读入棋盘格图像
        void imRead(const std::string path);
        // \brief 执行标定
        bool runCalibration();
        // \brief 将标定参数写入文件之中
        //bool calWrite();
        // \brief 标定结果评价
        bool evaluator();
    private:
        std::vector<fs::path> getBmpFiles(const fs::path&);
    };
} // namespace CQGCalibration
#endif // !CALIBRATION_H_