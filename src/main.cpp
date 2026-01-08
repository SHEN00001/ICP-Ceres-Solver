#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include "cost_function.hpp"

using namespace std;

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

int main(int argc, char *argv[]){
    string source_pcd = "../data/source.pcd";
    string target_pcd = "../data/target.pcd";
    
    PointCloud::Ptr source(new PointCloud);
    PointCloud::Ptr target(new PointCloud);

    pcl::io::loadPCDFile(source_pcd, *source);
    pcl::io::loadPCDFile(target_pcd, *target);

    // cout << "成功读取 Source 点数：" << source->size() << endl;
    // cout << "成功读取 Target 点数：" << target->size() << endl;

    // KD-Tree 快速查找target最近点
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target); // 要查找的就是target

    // 初始化待优化变量 优化后旋转：[0.93694, -0.00363866, 0.0100267, 0.349328], 优化后平移：[1.99804, 0.0563177, 0.017606]
    double q[4] = {cos(0.78/2), 0, 0, sin(0.78/2)};
    double t[3] = {2.0, 0.0, 0.0};

    // ICP 循环次数
    int interations = 100;
    for(int iter = 0; iter < interations; iter++){
        // --- 步骤 (1): 寻找匹配点 (Data Association) ---
        // 我们需要把 source 里的点，用当前的 q, t 变换一下，去 target 里找对应的点

        // 构建Ceres问题
        ceres::Problem problem;
        // 添加四元数约束块
        problem.AddParameterBlock(q, 4, new ceres::EigenQuaternionParameterization());
        problem.AddParameterBlock(t, 3);

        int match_count = 0;

        // 遍历pcd里的每一个点
        for(size_t i = 0; i<source->size(); i++){
            PointT p_origin = source->points[i];

            // 手动变换点
            Eigen::Quaterniond q_curr(q[0], q[1], q[2], q[3]);
            Eigen::Vector3d t_curr(t[0], t[1], t[2]);
            Eigen::Vector3d p_eigen(p_origin.x, p_origin.y, p_origin.z);

            // 变换 p_new = R * p + t
            Eigen::Vector3d p_new_eigen = q_curr * p_eigen + t_curr;
            PointT p_new;
            p_new.x = p_new_eigen[0];
            p_new.y = p_new_eigen[1];
            p_new.z = p_new_eigen[2];

            // 在target里找最近邻
            vector<int> pointIdx(1);
            vector<float> pointSqDist(1);
            // 调用 kdtree.nearestKSearch(查询点, K=1, 索引结果, 距离结果)
            if(kdtree.nearestKSearch(p_new, 1, pointIdx, pointSqDist)>0){ // 如果找到了最近邻的点（>0）
                // 如果距离小于阈值（例如 0.5米），认为匹配有效
                if(pointSqDist[0] < 0.5 * 0.5){
                    // 找到了！source[i] 对应 target[pointIdx[0]]
                    Eigen::Vector3d p_s = p_eigen;
                    // 目标点
                    PointT pt = target->points[pointIdx[0]];
                    Eigen::Vector3d p_t(pt.x, pt.y, pt.z);

                    // 此时两个Eigen向量格式的点p_s p_t准备好了，塞进计算模型结构体里
                    ceres::CostFunction* cost_function = PointToPointError::Create(p_s, p_t);
                    // 添加AddResidualBlock
                    problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), q, t);

                    match_count++;
                }
            }
        }

        cout << "第 " << iter << " 次迭代，找到匹配点: " << match_count << endl;

        // --- 步骤 (2): 求解 (Solve) ---
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 5;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        cout << "优化后旋转：[" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] 
             << "], 优化后平移：[" << t[0] << ", " << t[1] << ", " << t[2] << "]" << endl;
    }

    // 保存匹配后的点云
    Eigen::Quaterniond q_final(q[0], q[1], q[2], q[3]);
    Eigen::Vector3d t_final(t[0], t[1], t[2]);
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3,3>(0,0) = q_final.toRotationMatrix();
    transformation.block<3,1>(0,3) = t_final;

    PointCloud::Ptr source_transformed(new PointCloud);
    pcl::transformPointCloud(*source, *source_transformed, transformation);
    pcl::io::savePCDFileBinary("../data/source_aligned.pcd", *source_transformed);

    return 0;
}