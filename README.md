# Simple ICP Implementation using Ceres Solver

这是一个基于 C++ 实现的手写 ICP (Iterative Closest Point) 算法 demo。

本项目旨在演示如何结合 **PCL (Point Cloud Library)** 的数据处理能力与 **Ceres Solver** 的非线性优化能力，手动实现点云配准的核心流程。它不依赖 PCL 自带的 `IterativeClosestPoint` 接口，而是显式地构建了最小二乘问题。

## 📌 项目特性

* **手写优化模型**：使用 Ceres 定义 Point-to-Point 残差模型。
* **自动求导**：利用 Ceres 的 AutoDiff 机制计算雅可比矩阵。
* **流形优化**：使用 `EigenQuaternionParameterization` 处理四元数约束，防止旋转变形。
* **鲁棒核函数**：引入 Huber Loss 抑制误匹配带来的影响。
* **数据关联**：使用 PCL 的 KD-Tree 加速最近邻搜索。

## 📂 目录结构

```text
ICP-Ceres-Solver/
├── CMakeLists.txt          # 构建脚本
├── data/                   # 测试数据
│   ├── source.pcd          # 待配准点云
│   ├── target.pcd          # 目标点云
│   └── source_aligned.pcd  # (运行后生成) 配准结果
├── include/
│   └── cost_function.hpp   # Ceres 代价函数定义
└── src/
    └── main.cpp            # 主程序逻辑

```

## 🛠️ 依赖库

在编译之前，请确保你的系统已安装以下库：

* **CMake**
* **PCL** (Point Cloud Library)
* **Ceres Solver**
* **Eigen3**

## 🚀 编译与运行

1. **克隆或下载本项目**
2. **创建构建目录并编译**：
```bash
mkdir build
cd build
cmake ..
make

```


3. **运行程序**：
确保 `data` 目录下有 `source.pcd` 和 `target.pcd` 文件。
```bash
./icp_ceres_solver

```


终端将输出每次迭代的优化结果（旋转四元数与平移向量）。
4. **可视化结果**：
运行结束后，`data` 目录下会生成 `source_aligned.pcd`。可以使用 `pcl_viewer` 对比配准前后的效果：
```bash
# 对比 目标点云 和 配准后的源点云
pcl_viewer ../data/target.pcd ../data/source_aligned.pcd

```



## 🧠 核心算法逻辑

项目的核心逻辑遵循标准的 ICP 迭代步骤：

1. **数据关联 (Data Association)**：
* 将 Source 点云按照当前的位姿  变换到 Target 坐标系。
* 利用 KD-Tree 在 Target 中查找最近邻点。
* 根据距离阈值剔除误匹配。


2. **构建优化问题 (Optimization)**：
* 定义残差：
* 使用 Huber 核函数增强鲁棒性。
* 对四元数  施加流形约束（Local Parameterization）。


3. **求解与更新**：
* 调用 `ceres::Solve` 使用高斯-牛顿法或 LM 算法求解。
* 更新位姿，进入下一次迭代。



## ⚠️ 注意事项与局限性

* **初值敏感**：ICP 是局部优化算法，如果 Source 和 Target 初始位置相差过大（例如旋转 > 90°），极易陷入局部最优。建议在代码中提供一个粗略的初始猜测（Initial Guess）。
* **参数调试**：如果发现不收敛，尝试调整 `main.cpp` 中的迭代次数或 Huber Loss 的阈值。
