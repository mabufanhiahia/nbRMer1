#include <iostream>//引入输入输出函数
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
int main() {

    Point3D point3D = {0.5, 0.5, 1.0}; // 三维点
    CameraIntrinsic intrinsic = {500.0, 500.0, 320.0, 240.0}; // 相机内参
    CameraExtrinsic extrinsic = {Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.1, 0.1, 0.1)}; // 外参

    std::vector<double> pixelCoords;
    double reprojectionError;

    // 调用重投影函数
    std::vector<double> observedpix = {592.0, 512.0};
    // 【新增】自动化测试计数器
    int total_tests = 0;
    int passed_tests = 0;

    // ==========================================
    // 测试 1：正常投影 
    // ==========================================
    {
        total_tests++;
        std::cout << ">>> Test 1: Normal Projection & Perfect Alignment" << std::endl;
        
    
        point3D = {0.5, 0.5, 1.0}; 
        
        // 模拟完美测量
        double true_u = intrinsic.fx * ((point3D.x + extrinsic.t[0]) / (point3D.z + extrinsic.t[2])) + intrinsic.cx;
        double true_v = intrinsic.fy * ((point3D.y + extrinsic.t[1]) / (point3D.z + extrinsic.t[2])) + intrinsic.cy;
        observedpix = {true_u, true_v}; 

        try {
            reproject(point3D, intrinsic, extrinsic, pixelCoords, reprojectionError, observedpix);
            
            // 自动判分：误差应趋近于 0
            if (reprojectionError < 0.001) {
                std::cout << "  [PASS] Error is near zero: " << reprojectionError << std::endl;
                passed_tests++;
            } else {
                std::cout << "  [FAIL] Unexpected error: " << reprojectionError << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "  [FAIL] Should not throw exception: " << e.what() << std::endl;
        }
    }

    // ==========================================
    // 测试 2：异常边界测试 (Z <= 0)
    // ==========================================
    {
        total_tests++;
        std::cout << "\n>>> Test 2: Exception Handling (Z <= 0)" << std::endl;
        
        // 将点移到相机背后
        point3D = {0.0, 0.0, -1.0}; 
        observedpix = {320.0, 240.0};

        try {
            reproject(point3D, intrinsic, extrinsic, pixelCoords, reprojectionError, observedpix);
            // 如果代码执行到这里，说明没有抛出异常，测试失败
            std::cout << "  [FAIL] Function did not catch invalid Z value." << std::endl;
        } catch (const std::runtime_error& e) {
            // 成功捕获异常，测试通过
            std::cout << "  [PASS] Correctly caught exception: " << e.what() << std::endl;
            passed_tests++;
        }
    }

        // ==========================================
    // 测试 3：带噪声的观测值验证 
    // ==========================================
    {
        total_tests++;
        std::cout << "\n>>> Test 3: Noisy Observation Verification" << std::endl;
        
        point3D = {1.0, 1.0, 5.0};
        
        // 出当前外参下的理论投影点
        Eigen::Vector3d p_cam = extrinsic.R * Eigen::Vector3d(point3D.x, point3D.y, point3D.z) + extrinsic.t;
        double true_u = intrinsic.fx * (p_cam.x() / p_cam.z()) + intrinsic.cx;
        double true_v = intrinsic.fy * (p_cam.y() / p_cam.z()) + intrinsic.cy;
        
        // 构造一个偏离理论值 10 像素的观测点
        observedpix = {true_u + 10.0, true_v + 10.0}; 

        try {
            reproject(point3D, intrinsic, extrinsic, pixelCoords, reprojectionError, observedpix);
            
            // 预期误差sqrt(10^2 + 10^2) ≈ 14.14
            double expected_err = std::sqrt(200.0);
            
            if (std::fabs(reprojectionError - expected_err) < 0.01) {
                std::cout << "  [PASS] Error calculation correct: " << reprojectionError << std::endl;
                passed_tests++;
            } else {
                std::cout << "  [FAIL] Error mismatch. Expected ~" << expected_err << ", Got: " << reprojectionError << std::endl;
            }
        } catch (...) {
            std::cout << "  [FAIL] Unexpected crash." << std::endl;
        }
    }

    // 报告 
    std::cout << "\n========================================" << std::endl;
    std::cout << "AUTOMATED TEST REPORT" << std::endl;
    std::cout << "Total: " << total_tests << " | Passed: " << passed_tests << " | Failed: " << (total_tests - passed_tests) << std::endl;
    if (passed_tests == total_tests) {
        std::cout << ">> ALL TESTS PASSED! Algorithm is robust. <<" << std::endl;
    } else {
        std::cout << ">> SOME TESTS FAILED. Please review logic. <<" << std::endl;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}