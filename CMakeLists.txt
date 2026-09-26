# 1. CMake 3.10 版本
cmake_minimum_required(VERSION 3.10)

# 2. 给项目起个名，叫 CameraProjection
project(CameraProjection LANGUAGES CXX)

# 3. 我们要用 C++11 标准写代码
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 4. 头文件(.h)都藏在 include 文件夹里
include_directories(${PROJECT_SOURCE_DIR}/include)

# 5. 帮我找找系统里有没有装 Eigen3 库
find_package(Eigen3 REQUIRED)

# 6. 把 src 和 tests 里的代码编译成一个叫 test_proj 的程序
add_executable(test_proj
    src/camera_model.cpp
    tests/test_projection.cpp
)

# 7. 把 Eigen3 库的功能接给 test_proj 程序用
target_link_libraries(test_proj PRIVATE Eigen3::Eigen)