#ifndef COST_FUNCTION_HPP
#define COST_FUNCTION_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

struct PointToPointError
{
    PointToPointError(Eigen::Vector3d p_src, Eigen::Vector3d p_tgt) : p_src_(p_src), p_tgt_(p_tgt){}

    template <typename T>
    bool operator()(const T* const q, const T* const t, T* residual) const
    {
        // 1.准备数据
        T p_s[3] = {T(p_src_[0]), T(p_src_[1]), T(p_src_[2])};
        T p_final[3];
        
        // 2.旋转 p_final = R(q) * q_s
        // Ceres工具：ceres::QuaternionRotatePoint(q, p_in, p_out)
        ceres::QuaternionRotatePoint(q, p_s, p_final);

        // 3.平移 p_final = p + t
        p_final[0] += t[0];
        p_final[1] += t[1];
        p_final[2] += t[2];

        // 4.计算误差：residual = p_final - p_tgt_
        residual[0] = p_final[0] - T(p_tgt_[0]);
        residual[1] = p_final[1] - T(p_tgt_[1]);
        residual[2] = p_final[2] - T(p_tgt_[2]);
        
        return true;
    }

    // 工厂模式：方便在 main 函数里创建
    static ceres::CostFunction* Create(const Eigen::Vector3d& p_src, const Eigen::Vector3d& p_tgt) {
        // <仿函数类型, 残差维度3, 参数块1维度(q)4, 参数块2维度(t)3>
        // 这一行我已经帮你写好了，不用改，知道意思就行
        return (new ceres::AutoDiffCostFunction<PointToPointError, 3, 4, 3>(
            new PointToPointError(p_src, p_tgt)));
    }

    const Eigen::Vector3d p_src_, p_tgt_;
};



#endif COST_FUNCTION_HPP