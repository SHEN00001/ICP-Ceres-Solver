cmake_minimum_required(VERSION 3.16)

project(ICP-Ceres-Solver LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找依赖库
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# 包含头文件路径
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_SIR}
)

# 设置相关定义
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARIES_DIRS})

# 生成可执行文件
add_executable(icp_ceres_solver src/main.cpp)

# 链接库文件
target_link_libraries(icp_ceres_solver
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    Eigen3::Eigen
)

add_executable(point_to_plane src/point_to_plane_residual.cpp)
target_link_libraries(point_to_plane
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    Eigen3::Eigen
)

add_executable(plane_fitting src/plane_fitting.cpp)
target_link_libraries(plane_fitting
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    Eigen3::Eigen
)