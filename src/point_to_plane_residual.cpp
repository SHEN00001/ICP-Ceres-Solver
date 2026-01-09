#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

#include <cost_function.hpp>

using namespace std;

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;

class PointToPlane
{
public:
    PointToPlane();
    ~PointToPlane();
    void SavePcd();

private:
    string source_pcd = "../data/source.pcd";
    string target_pcd = "../data/target.pcd";
    string source_transformed_pcd = "../data/point_to_plane.pcd";

    PointCloud::Ptr source_, target_;
    PointCloud::Ptr source_transformed_;

    double q_[4] = {cos(0.78/2), 0, 0, sin(0.78/2)}; // w, x, y, z
    double t_[3] = {2.0, 0.0, 0.0};

    Eigen::Quaterniond q_fianl_;
    Eigen::Vector3d t_final_;

};

PointToPlane::PointToPlane(){
    source_ = PointCloud::Ptr(new PointCloud);
    target_ = PointCloud::Ptr(new PointCloud);

    pcl::io::loadPCDFile(source_pcd, *source_);
    pcl::io::loadPCDFile(target_pcd, *target_);

    // cout << "成功读取source点云, 点数: " << source_->size() << endl;
    // cout << "成功读取target点云, 点数: " << target_->size() << endl;

    // 估计target点云的法向量
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(target_);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>());
    ne.setKSearch(10);
    ne.compute(*target_normals);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target_);

    int interations = 50;
    for(int iter = 0; iter < interations; iter++){
        ceres::Problem problem;
        problem.AddParameterBlock(q_, 4, new ceres::EigenQuaternionParameterization());
        problem.AddParameterBlock(t_, 3);

        int match_count = 0;

        for(int i = 0; i<source_->size(); i++){
            PointT p = source_->points[i]; // pcl格式的点
            Eigen::Vector3d p_eigen(p.x, p.y, p.z); // Eigen格式的点

            Eigen::Quaterniond q_eigen(q_[0], q_[1], q_[2], q_[3]);
            Eigen::Vector3d t_eigen(t_[0], t_[1], t_[2]);

            Eigen::Vector3d p_new_eigen = q_eigen * p_eigen + t_eigen;

            PointT p_new;
            p_new.x = p_new_eigen[0];
            p_new.y = p_new_eigen[1];
            p_new.z = p_new_eigen[2];

            vector<int> pointIdx(1);
            vector<float> pointSqDist(1);
            if(kdtree.nearestKSearch(p_new, 1, pointIdx, pointSqDist)>0){
                if(pointSqDist[0] < 0.5*0.5){
                    auto n = target_normals->points[pointIdx[0]];
                    Eigen::Vector3d norm_vec(n.normal_x, n.normal_y, n.normal_z);

                    // 如果法向量无效（比如是NaN），就跳过
                    if(!isfinite(norm_vec.x()) || !isfinite(norm_vec.y()) || !isfinite(norm_vec.z())){
                        continue;
                    }

                    Eigen::Vector3d p_s = p_eigen;
                    PointT pt = target_->points[pointIdx[0]];
                    Eigen::Vector3d p_t(pt.x, pt.y, pt.z);

                    // 代价函数
                    ceres::CostFunction* cost_function = PointToPlaneError::Create(p_s, p_t, norm_vec);
                    problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1), q_, t_);
                    
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

        cout << "优化后旋转：[" << q_[0] << ", " << q_[1] << ", " << q_[2] << ", " << q_[3] 
             << "], 优化后平移：[" << t_[0] << ", " << t_[1] << ", " << t_[2] << "]" << endl;
    }
}

PointToPlane::~PointToPlane(){}


void PointToPlane::SavePcd(){
    q_fianl_.w() = q_[0];
    q_fianl_.x() = q_[1];
    q_fianl_.y() = q_[2];
    q_fianl_.z() = q_[3];

    t_final_[0] = t_[0];
    t_final_[1] = t_[1];
    t_final_[2] = t_[2];

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3,3>(0,0) = q_fianl_.toRotationMatrix();
    transformation.block<3,1>(0,3) = t_final_;

    source_transformed_ = PointCloud::Ptr(new PointCloud);
    pcl::transformPointCloud(*source_, *source_transformed_, transformation);
    pcl::io::savePCDFileBinary(source_transformed_pcd, *source_transformed_);
}


int main(int argc, char *argv[]){
    PointToPlane point2plane;
    point2plane.SavePcd();
    return 0;
}