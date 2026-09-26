#include<camera_model.h>
#include <iostream>
#include<cmath>
// 重投影函数reproject
void reproject(const Point3D& point3D, const CameraIntrinsic& intrinsic, const CameraExtrinsic& extrinsic,
std::vector<double>& pixelCoords, double& reprojectionError,std::vector<double>&observedpix) {
    // 从世界坐标系到相机坐标系的转换
    Eigen::Vector3d point3DCamera = extrinsic.R * Eigen::Vector3d(point3D.x, point3D.y, point3D.z) + extrinsic.t;
    //Pc = R*Pw + t
    if (point3DCamera.z() <= 0) {
        throw std::runtime_error("！！！！！Point is behind or on the camera plane (z <= 0)！！！！！");
    }
    // 透视投影
    //把相机坐标系下的3D点转换成照片上的2D像素点
    Eigen::Vector3d normalizedPoint = point3DCamera / point3DCamera.z();
    double u = normalizedPoint.x() * intrinsic.fx + intrinsic.cx;
    double v = normalizedPoint.y() * intrinsic.fy + intrinsic.cy;

    pixelCoords = {u, v};
    double error_u =u - observedpix[0];
    double error_v =v - observedpix[1];
    reprojectionError = std::sqrt(std::pow(error_u, 2) + std::pow(error_v, 2));
}