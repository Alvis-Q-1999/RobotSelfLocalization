# 基于粒子群优化与粒子滤波算法的移动机器人自定位实验

本仓库包含了基于 **粒子群优化 (Particle Swarm Optimization, PSO)** 与 **粒子滤波 Particle Filter（PF）** 的移动机器人自定位算法实现。  
机器人在二维栅格地图中，通过模拟激光扫描 (8 个方向) 获取环境特征，结合群体智能优化与聚类方法实现位置估计与姿态追踪。  


## 仓库结构
PSO_PF/

├── core/ # 核心算法函数（PSO、DBSCAN、重采样、扫描等）

├── viz/ # 可视化函数（绘制地图、粒子、真实位置等）

├── map/ # 实验地图（.mat 格式）

├── examples/ # 示例入口（main.m 等）

├── docs/ # 文档与实验结果

│ └── videos/ # 实验录像存放处

└── legacy/ # 原始版本代码（未模块化）

##  算法概览

- **环境表征**：二值栅格地图 `G`（`0=空地`，`1=障碍`）。
- **观测模型**：`scan8dirs` 在 8 个方向上扫描到障碍/边界为止，得到 8 维向量（“可行距离”）。
- **适配衡量**：与真实扫描向量 `real_scan` 的余弦相似度 `fitness`。
- **PSO 初始定位**：  
  - 个体最优 `P_best` 与“邻域最优候选”`Top_best_pos` 协同更新；  
  - 速度/位置剪裁；  
  - 只接受落在空地的更新，避免穿墙。
- **重采样**：与最近邻域最优的距离超过阈值的粒子，直接重采样到该邻域最优位置处。
- **DBSCAN 聚类**：将粒子聚类为若干子群（噪点复制到随机簇成员），用于稳定收敛与后续子群级重采样。
- **动态阶段**：  
  - **自主移动（推荐）**：先选最空旷方向，随后在包含该方向的**锥形邻域（如 4 个相邻方向）**内按可行距离加权随机游走；  
  - 粒子随动（逐轴推进 + 随机扰动，遇墙/边界即停）；  
  - 子群内部评估与对齐；  
  - 二次 DBSCAN；  
  - 收敛到单簇即终止。


## 关键参数
- **PSO**：w=0.87, c1=0.5, c2=0.5；速度与位置剪裁由 clamp 实现。

- **邻域最优候选**：Top_best_pos 每 10 次迭代更新一次、阈值逐步收紧。

- **重采样**：离最近邻域最优距离超过阈值（如 2）的粒子直接对齐到簇心。

- **DBSCAN**：初次：epsilon=3, minPts=20；
动态二次：epsilon=2, minPts=50；


## 实验结果
**回字型**
https://github.com/Alvis-Q-1999/RobotSelfLocalization/blob/main/PSO_PF/result/H1_pic.png

**走廊型**
https://github.com/Alvis-Q-1999/RobotSelfLocalization/blob/main/PSO_PF/result/L1_pic.png

**随机型**
https://github.com/Alvis-Q-1999/RobotSelfLocalization/blob/main/PSO_PF/result/R1_pic.png

**障碍型**
https://github.com/Alvis-Q-1999/RobotSelfLocalization/blob/main/PSO_PF/result/O1_pic.png
