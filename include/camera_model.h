#pragma once
#include <vector>//引入动态数组，储存重投影后的2D像素坐标
#include <Eigen/Dense>//引入矩阵模块
struct Point3D {
    double x, y, z;//定义一个3D点，可以表示三维世界的坐标
};
//定义相机内参 
struct CameraIntrinsic {
    double fx, fy; // 焦距
    double cx, cy; // 主点
};   
//定义相机外参
struct CameraExtrinsic {
    Eigen::Matrix3d R; // 旋转矩阵
    Eigen::Vector3d t; // 平移向量
};
// 重投影函数reproject
void reproject(const Point3D& point3D, const CameraIntrinsic& intrinsic, const CameraExtrinsic& extrinsic,
std::vector<double>& pixelCoords, double& reprojectionError,std::vector<double>&observedpix);