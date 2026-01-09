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


struct PointToPlaneError
{
    PointToPlaneError(Eigen::Vector3d p_src, Eigen::Vector3d p_tgt, Eigen::Vector3d plane_unit_norm) : 
                      p_src_(p_src), p_tgt_(p_tgt), plane_unit_norm_(plane_unit_norm){}

    template <typename T>
    bool operator()(const T* const q, const T* const t, T* residual) const 
    {
        T p_s[3] = {T(p_src_[0]), T(p_src_[1]), T(p_src_[2])};
        T p_t[3] = {T(p_tgt_[0]), T(p_tgt_[1]), T(p_tgt_[2])};
        T p_trans[3];

        ceres::QuaternionRotatePoint(q, p_s, p_trans);
        p_trans[0] += t[0];
        p_trans[1] += t[1];
        p_trans[2] += t[2];

        // 计算 p_t 到 p_trans 的向量 d
        T d[3];
        d[0] = p_trans[0] - p_t[0];
        d[1] = p_trans[1] - p_t[1];
        d[2] = p_trans[2] - p_t[2];

        // residual：最小化向量d和单位法向量之间的内积（不考虑方向），如果内积等于0，则说明向量d长度为0，我们要的就是d为0
        residual[0] = d[0] * T(plane_unit_norm_[0]) + d[1] * T(plane_unit_norm_[1]) + d[2] * T(plane_unit_norm_[2]);

        return true;
    }

    // 依旧是Create函数
    static ceres::CostFunction* Create(const Eigen::Vector3d& p_src, const Eigen::Vector3d& p_tgt, const Eigen::Vector3d& plane_unit_norm){
        return (new ceres::AutoDiffCostFunction<PointToPlaneError, 1, 4, 3>(
            new PointToPlaneError(p_src, p_tgt, plane_unit_norm)
        ));
    }

    const Eigen::Vector3d p_src_, p_tgt_, plane_unit_norm_;
};



#endif COST_FUNCTION_HPP