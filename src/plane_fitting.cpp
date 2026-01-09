#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "cost_function.hpp"

using namespace std;

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

class PlaneFitting
{
private:
    string target_pcd_ = "../data/target.pcd";
    // string target_filtered_pcd_ = "../data/target_filtered.pcd";
    string result_pcd_ = "../data/result.pcd";

    double n_[3], d_[1];

    PointCloud::Ptr target_;
    PointCloud::Ptr target_filtered_;

public:
    PlaneFitting();
    ~PlaneFitting();

    void SavePcd();
};

PlaneFitting::PlaneFitting()
{
    // -------------------------------------------------------
    // 读取pcd
    // -------------------------------------------------------
    target_ = PointCloud::Ptr(new PointCloud);
    target_filtered_ = PointCloud::Ptr(new PointCloud);

    pcl::io::loadPCDFile(target_pcd_, *target_);

    // cout << "target.pcd的点数为: " << target_->size() << endl;
    
    // -------------------------------------------------------
    // filter 只保留 $z < min\_z + 1.0$（地面以上1米内）的点参与优化
    // -------------------------------------------------------
    // float min_z = 1e9;
    // for(auto& p : target_->points){
    //     min_z = min(min_z, p.z);
    // }
    // cout << "The minimum z-value is " << min_z << endl;
    float min_z = -1.3517; // 已经读取出来了，不用再每次执行都再算一遍了

    for(auto& p : target_->points){
        if(p.z < min_z + 0.3){
            target_filtered_->push_back(p);
        }
    }

    // -------------------------------------------------------
    // 初始化待优化变量
    // -------------------------------------------------------
    n_[0] = 0.0; // 单位法向量初始化
    n_[1] = 0.0;
    n_[2] = 1.0;
    d_[0] = -min_z;

    // -------------------------------------------------------
    // 构建Ceres问题
    // -------------------------------------------------------
    ceres::Problem problem;
    problem.AddParameterBlock(n_, 3, new ceres::HomogeneousVectorParameterization(3));
    problem.AddParameterBlock(d_, 1);

    for(const auto& p : target_filtered_->points){
        Eigen::Vector3d p_eigen(p.x, p.y, p.z);
        ceres::CostFunction* cost_function = PlaneFittingError::Create(p_eigen);
        problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), n_, d_);
    }


    // -------------------------------------------------------
    // 求解
    // -------------------------------------------------------
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 50;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    cout << summary.BriefReport() << endl;
    cout << "拟合结果: " << endl;
    cout << "法向量 n: [" << n_[0] << ", " << n_[1] << ", " << n_[2] << "]" << endl;
    cout << "截距 d: " << d_[0] << endl;
}

PlaneFitting::~PlaneFitting()
{
}


void PlaneFitting::SavePcd(){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(auto& p : target_->points){
        pcl::PointXYZRGB p_rgb;
        p_rgb.x = p.x;
        p_rgb.y = p.y;
        p_rgb.z = p.z;

        double dis = n_[0] * p.x + n_[1] * p.y + n_[2] * p.z + d_[0];

        if(abs(dis) < 0.1){
            p_rgb.r = 255;
            p_rgb.g = 0;
            p_rgb.b = 0;
        } else {
            p_rgb.r = 255;
            p_rgb.g = 255;
            p_rgb.b = 255;
        }
        result_cloud->points.push_back(p_rgb);
    }


    // pcl::io::savePCDFileBinary(target_filtered_pcd_, *target_filtered_);
    pcl::io::savePCDFileBinary(result_pcd_, *result_cloud);
}


int main(int argc, char *argv[]){
    PlaneFitting planeFitting;
    planeFitting.SavePcd();
    return 0;
}