# CDPRs_and_serialrobot_system

基于 **MATLAB / Simulink** 开发的 **缆索驱动并联机器人（Cable-Driven Parallel Robot, CDPR）与串联机械臂联合仿真平台**。项目围绕“绳索悬挂机械臂系统”的建模、规划与控制展开，集成了静力学建模、运动学建模、动力学建模、索力分布、力可行空间、最大负载分析、T 型轨迹规划、PID / 模糊 PID 控制与 Simulink 仿真验证等模块。

本仓库可作为 CDPR、绳索悬挂机械臂、移动平台-机械臂复合系统建模与控制研究的 MATLAB 工具链，也适用于机器人算法开发、仿真验证、课程设计和科研原型实现。

---

## 项目简介

绳索悬挂机械臂系统由 **缆索驱动并联机器人（CDPR）** 和 **串联机械臂** 组成。CDPR 通过多根绳索驱动动平台运动，具有工作空间大、负载能力强、结构轻量化等特点；串联机械臂安装在动平台上，用于完成更复杂、更灵活的末端操作任务。

本项目主要实现以下功能：

- CDPR 力可行空间（Wrench Feasible Workspace, WFW）计算；
- 系统最大负载能力分析；
- CDPR 正 / 逆运动学求解；
- 串联机械臂正 / 逆运动学建模；
- CDPR 与串联机械臂动力学建模；
- 多缆索索力分布与张力合法性判断；
- 三维空间 T 型轨迹规划；
- 基于前馈补偿的关节空间 / 笛卡尔空间 PID 控制；
- 模糊 PID 控制器仿真；
- MATLAB / Simulink 控制系统搭建与仿真验证。

---

## 主要特点

- **全流程仿真平台**：覆盖性能评估、建模、规划、控制与结果分析。
- **面向 CDPR + 串联机械臂复合系统**：既包含 CDPR 独立算法，也包含串联机械臂相关模块。
- **力可行性分析**：基于正交投影法判断广义力矩是否满足绳索张力上下限约束。
- **索力分布优化**：实现适用于 `k+2` 根绳索驱动 `k` 自由度 CDPR 的索力分布算法。
- **运动学求解完整**：CDPR 正运动学采用区间初值估计 + Levenberg-Marquardt 迭代求解，逆运动学基于矢量闭合关系直接计算。
- **动力学建模与前馈补偿**：基于牛顿-欧拉递推法建立 CDPR 与串联机械臂动力学模型，并用于控制前馈补偿。
- **Simulink 集成**：包含多个 `.slx` 控制系统模型，便于模块化仿真和控制方案验证。
- **结果可视化**：包含大量 `.fig` 仿真结果图，可用于分析力可行空间、负载能力、轨迹跟踪误差和张力曲线。

---

## 系统方法概览

```text
系统参数建模
    ↓
CDPR 静力学建模
    ↓
力可行空间 / 最大负载能力分析
    ↓
CDPR 与串联机械臂运动学建模
    ↓
CDPR 与串联机械臂动力学建模
    ↓
索力分布求解
    ↓
三维 T 型轨迹规划
    ↓
前馈补偿 + PID / 模糊 PID 控制
    ↓
MATLAB / Simulink 仿真验证
```

---

## 项目目录结构

```text
CDPRs_and_serialrobot_system/
├── 1.力可行空间相关/
│   ├── PM_Method_Func.m
│   ├── Projection_Method_tset.m
│   ├── Nolive_payload.m
│   ├── plot_cuboid.m
│   ├── plotkuangjia.m
│   ├── Theta2RotationMatrix_Func.m
│   ├── Nolivepayload1.mat
│   └── 图*.fig
│
├── 2.机械臂最大负载相关/
│   ├── Maximun_payload.m
│   ├── Nolive_payload.m
│   ├── PM_Method_Func.m
│   ├── Theta2RotationMatrix_Func.m
│   └── 图*.fig
│
├── 3.运动学相关/
│   ├── ForwardKenamatics_Func.m
│   ├── InverseKenematics_Func.m
│   ├── IKandFK.m
│   ├── IKandFK_calc_time.m
│   ├── Inverse_Dynamics_Func.m
│   ├── Jaco.m
│   ├── Q_criteria.m
│   ├── FunctionVector_f.m
│   └── *.mat / *.fig
│
├── 4.索力分布相关/
│   ├── Tension_Distribution_Func.m
│   ├── Tension_Distribution_Calc.m
│   ├── Tension_Distribution2_Func.m
│   ├── Tension_Distribution3_Func.m
│   ├── Tension_Distribution4_Func.m
│   ├── Judge_Tension_Distribution_Func.m
│   ├── Tension_Distribution_Test*.m
│   ├── Theta2RotationMatrix_Func.m
│   └── WFW*.fig / 图*.fig
│
├── 5.动力学相关/
│   ├── Dynamic_Maximun_payload.m
│   ├── Inverse_Dynamics_Func.m
│   ├── Inverse_Jacobian_Func.m
│   ├── Jacobian_Func.m
│   ├── Twolinks_Dynamic_Func.m
│   ├── Twolinks_Dynamics_Test.m
│   └── Jacobian_and_InverseJacobian_Test.m
│
├── 6.T型轨迹规划相关/
│   ├── Path_Planning_plot.m
│   ├── Path_Planning_SFunc.m
│   ├── Path_Planning_SFunc_Simulink.slx
│   ├── Path_Planning_Subsystem_Simulink.slx
│   ├── Theta2RotationMatrix_Func.m
│   └── 图*.fig
│
├── 7.控制部分各模块仿真测试/
│   ├── SCDPR_Func.m
│   ├── Path_Planning_SFunc.m
│   ├── Inverse_Dynamics_Func_Simulink.slx
│   ├── Jacobian_and_IJacobian_Func_Simulink.slx
│   ├── Kenematics_and_IKenematics_Func_Simulink.slx
│   ├── Tension_Distribution_Func_Simulink.slx
│   ├── ode45_Test.m
│   └── slprj/
│
├── Control_Scheme/
│   ├── Control_Scheme_Cartesian_PID_Simulink.slx
│   ├── Control_Scheme_Cartesian_FuzzPID_Simulink.slx
│   ├── Control_Scheme_Joint_PID_Simulink.slx
│   ├── Fuzz_PID_Sys.slx
│   ├── fuzzyPID.fis
│   ├── Neur_PID.m
│   ├── Neur_PID_sys.slx
│   ├── Path_Planning_SFunc.m
│   ├── FunctionVector_f.m
│   └── 相关 Simulink 生成文件
│
├── 串联机械臂相关/
│   ├── DH_2_Trans.m
│   ├── ikine3R.m
│   ├── RRR_TEST.m
│   ├── UR3.m
│   ├── RZYX.m
│   ├── RZYX_2_T.m
│   ├── Rev_RZYX.m
│   ├── RTB.mltbx
│   └── eigen-3.4.0/
│
├── 求解正交投影的所有组合/
│   ├── Get_all_choice.cpp
│   └── Get_all_choice.exe
│
├── CDPRs_and_serialrobot_system.code-workspace
└── LICENSE
```

---

## 模块说明

### 1. 力可行空间相关

该模块用于计算 CDPR 的 **力可行工作空间（Wrench Feasible Workspace, WFW）**。核心目标是判断在给定位姿下，CDPR 是否能够通过满足张力约束的绳索力平衡外部广义力矩。

主要功能：

- 建立 CDPR 静力学结构矩阵；
- 基于正交投影法判断广义力矩可行性；
- 遍历离散工作空间点；
- 可视化力可行空间；
- 评估动平台姿态变化对可行空间的影响。

关键文件：

| 文件 | 说明 |
|---|---|
| `PM_Method_Func.m` | 正交投影法核心函数 |
| `Projection_Method_tset.m` | 正交投影法测试程序 |
| `Nolive_payload.m` | 空载状态下力可行空间计算 |
| `plot_cuboid.m` | 工作空间立方体绘制 |
| `plotkuangjia.m` | CDPR 框架绘制 |
| `Theta2RotationMatrix_Func.m` | 欧拉角到旋转矩阵转换 |

---

### 2. 机械臂最大负载相关

该模块用于分析在给定 CDPR 动平台位姿下，串联机械臂在不同构型或工作空间点处的 **最大安全负载能力**。

主要功能：

- 根据串联机械臂构型计算外部广义力矩；
- 结合张力上下限判断负载是否可行；
- 逐步增加负载质量，搜索最大可承载质量；
- 可视化不同平台姿态下的负载能力分布。

关键文件：

| 文件 | 说明 |
|---|---|
| `Maximun_payload.m` | 最大负载计算主程序 |
| `Nolive_payload.m` | 空载 / 负载条件下外力矩计算 |
| `PM_Method_Func.m` | 负载可行性判断 |
| `Theta2RotationMatrix_Func.m` | 姿态转换工具函数 |

---

### 3. 运动学相关

该模块实现 CDPR 与串联机械臂的正 / 逆运动学求解。

CDPR 部分：

- 逆运动学：根据动平台位姿直接计算各绳索长度；
- 正运动学：根据绳长反解动平台位姿；
- 使用区间法获得初始位姿；
- 使用 Levenberg-Marquardt 算法迭代求解非线性最小二乘问题；
- 计算雅可比矩阵与运动学评价指标。

串联机械臂部分：

- 基于 DH 参数建立正运动学模型；
- 支持 UR3 机械臂运动学验证；
- 支持逆运动学计算与仿真。

关键文件：

| 文件 | 说明 |
|---|---|
| `ForwardKenamatics_Func.m` | CDPR 正运动学求解 |
| `InverseKenematics_Func.m` | CDPR 逆运动学求解 |
| `IKandFK.m` | 正逆运动学综合测试 |
| `IKandFK_calc_time.m` | 运动学算法耗时统计 |
| `Jaco.m` | 雅可比矩阵计算 |
| `Q_criteria.m` | 运动学性能评价指标 |
| `FunctionVector_f.m` | LM 迭代目标函数相关计算 |

---

### 4. 索力分布相关

该模块用于 CDPR 的多缆索张力分配，是系统动力学控制中的核心部分。由于 CDPR 绳索只能产生拉力，索力分布算法需要保证每根绳索张力满足：

```text
t_min ≤ t_i ≤ t_max
```

主要功能：

- 给定位姿与广义力矩，求解可行索力；
- 判断索力是否松弛或超过上限；
- 实现多版本索力分布算法；
- 使用凸多边形质心方法获得具有较大裕度的索力解；
- 可用于替代正交投影法进行部分力可行空间计算。

关键文件：

| 文件 | 说明 |
|---|---|
| `Tension_Distribution_Func.m` | 索力分布主函数 |
| `Tension_Distribution_Calc.m` | 索力数值计算 |
| `Tension_Distribution2_Func.m` | 索力分布算法版本 2 |
| `Tension_Distribution3_Func.m` | 索力分布算法版本 3 |
| `Tension_Distribution4_Func.m` | 索力分布算法版本 4 |
| `Judge_Tension_Distribution_Func.m` | 索力合法性判断 |
| `Tension_Distribution_Test*.m` | 索力分布测试程序 |

---

### 5. 动力学相关

该模块实现 CDPR 与串联机械臂的动力学建模与验证。

主要功能：

- 建立 CDPR 动力学模型；
- 建立串联机械臂牛顿-欧拉动力学模型；
- 建立简化系统动力学模型；
- 实现逆动力学计算；
- 计算雅可比矩阵与逆雅可比矩阵；
- 分析动态最大负载能力；
- 用于控制前馈补偿和系统仿真。

关键文件：

| 文件 | 说明 |
|---|---|
| `Inverse_Dynamics_Func.m` | 逆动力学计算 |
| `Dynamic_Maximun_payload.m` | 动态最大负载分析 |
| `Jacobian_Func.m` | 雅可比矩阵 |
| `Inverse_Jacobian_Func.m` | 逆雅可比矩阵 |
| `Twolinks_Dynamic_Func.m` | 两连杆机械臂动力学 |
| `Twolinks_Dynamics_Test.m` | 两连杆动力学测试 |

---

### 6. T 型轨迹规划相关

该模块实现三维空间下的 **T 型速度曲线轨迹规划**。

T 型轨迹规划用于生成平滑的加减速路径，速度曲线通常包括：

```text
匀加速 → 匀速 → 匀减速
```

如果路径长度不足以达到最大速度，则自动退化为三角速度曲线。

主要功能：

- 输入起点、终点、最大速度、最大加速度；
- 自动判断梯形速度曲线或三角速度曲线；
- 输出位置、速度、加速度轨迹；
- 支持三维直线轨迹生成；
- 提供 MATLAB 函数与 Simulink S-Function 实现。

关键文件：

| 文件 | 说明 |
|---|---|
| `Path_Planning_plot.m` | T 型轨迹规划绘图 |
| `Path_Planning_SFunc.m` | S-Function 轨迹规划实现 |
| `Path_Planning_SFunc_Simulink.slx` | Simulink 轨迹规划模型 |
| `Path_Planning_Subsystem_Simulink.slx` | Simulink 子系统模型 |

---

### 7. 控制部分各模块仿真测试

该模块用于对控制系统中的各个子模块进行单独验证，包括运动学、动力学、雅可比、索力分布和轨迹规划模块。

主要功能：

- 单模块仿真测试；
- Simulink 功能块验证；
- 轨迹跟踪仿真；
- 误差曲线、张力曲线、位置曲线可视化；
- 为完整控制系统集成提供基础。

关键文件：

| 文件 | 说明 |
|---|---|
| `SCDPR_Func.m` | 绳索悬挂机械臂系统相关函数 |
| `Inverse_Dynamics_Func_Simulink.slx` | 逆动力学 Simulink 模块 |
| `Jacobian_and_IJacobian_Func_Simulink.slx` | 正 / 逆雅可比 Simulink 模块 |
| `Kenematics_and_IKenematics_Func_Simulink.slx` | 正 / 逆运动学 Simulink 模块 |
| `Tension_Distribution_Func_Simulink.slx` | 索力分布 Simulink 模块 |
| `ode45_Test.m` | 微分方程求解测试 |

---

### 8. Control_Scheme

该目录包含完整控制方案的 Simulink 实现，是项目的核心控制系统部分。

主要控制方案：

- 关节空间 PID 控制；
- 笛卡尔空间 PID 控制；
- 基于前馈补偿的双空间 PID 控制；
- 模糊 PID 控制；
- 神经网络 PID 相关测试；
- 轨迹规划与控制系统联合仿真。

关键文件：

| 文件 | 说明 |
|---|---|
| `Control_Scheme_Joint_PID_Simulink.slx` | 关节空间 PID 控制方案 |
| `Control_Scheme_Cartesian_PID_Simulink.slx` | 笛卡尔空间 PID 控制方案 |
| `Control_Scheme_Cartesian_FuzzPID_Simulink.slx` | 笛卡尔空间模糊 PID 控制方案 |
| `Fuzz_PID_Sys.slx` | 模糊 PID 控制器 |
| `fuzzyPID.fis` | 模糊控制规则文件 |
| `Neur_PID.m` | 神经网络 PID 测试代码 |
| `Neur_PID_sys.slx` | 神经网络 PID Simulink 模型 |

---

### 9. 串联机械臂相关

该模块包含串联机械臂建模、UR3 参数、DH 变换和逆运动学等内容。

主要功能：

- 标准 DH 建模；
- UR3 机械臂运动学建模；
- RZYX 欧拉角与齐次变换转换；
- 三自由度机械臂逆运动学测试；
- Robotics Toolbox 相关支持。

关键文件：

| 文件 | 说明 |
|---|---|
| `DH_2_Trans.m` | DH 参数转齐次变换矩阵 |
| `UR3.m` | UR3 机械臂建模 |
| `ikine3R.m` | 3R 机械臂逆运动学 |
| `RRR_TEST.m` | RRR 机械臂测试 |
| `RZYX.m` / `RZYX_2_T.m` | 欧拉角与变换矩阵相关函数 |
| `RTB.mltbx` | Robotics Toolbox 安装包 |

---

### 10. 求解正交投影的所有组合

该目录提供正交投影法中所需的组合求解工具。

正交投影法需要枚举结构矩阵中的若干列组合。该目录中的 C++ 程序用于生成所有可能组合，提高 MATLAB 主程序的计算效率。

| 文件 | 说明 |
|---|---|
| `Get_all_choice.cpp` | 组合生成源码 |
| `Get_all_choice.exe` | 编译后的可执行程序 |

---

## 主要算法说明

### 1. 正交投影法：力可行性判断

CDPR 的静力学平衡可表示为：

```text
W(x) t + w_e = 0
```

其中：

- `W(x)` 为结构矩阵；
- `t` 为绳索张力向量；
- `w_e` 为外部广义力矩。

由于绳索只能受拉，张力必须满足：

```text
t_min ≤ t_i ≤ t_max
```

正交投影法用于判断在给定位姿与外力矩下，是否存在满足张力约束的索力解。如果存在，则该位姿属于力可行空间。

---

### 2. 最大负载能力分析

在给定 CDPR 动平台位姿后，遍历串联机械臂工作空间内的构型，并逐步增加末端负载质量。每次负载增加后，根据系统静力学模型计算外部广义力矩，再通过正交投影法判断可行性。临界可行负载即为该位置的最大负载能力。

---

### 3. CDPR 正 / 逆运动学

CDPR 逆运动学通过矢量闭合关系直接计算绳长：

```text
l_i = a_i - r - R b_i
```

CDPR 正运动学由于绳索冗余，一般没有简单闭式解。本项目采用：

```text
区间估计初始值 + Levenberg-Marquardt 迭代
```

求解动平台位姿，使计算绳长与测量绳长之间的误差最小。

---

### 4. 索力分布算法

在给定位姿和广义力矩时，索力分布问题可以转化为寻找满足张力上下限的索力向量。对于 `k+2` 根绳索驱动 `k` 自由度的 CDPR，索力解空间可以映射到二维参数空间中。

本项目通过几何算法寻找二维可行凸多边形，并取其质心作为较优解，再映射回高维索力空间，得到具有较大安全裕度的张力分布。

---

### 5. 三维 T 型轨迹规划

T 型轨迹规划用于在起点和终点之间生成平滑轨迹。算法根据路径长度判断速度曲线类型：

- 路径足够长：梯形速度曲线；
- 路径较短：三角速度曲线。

三维规划中，先计算起点到终点的方向向量，再对路径长度进行一维 T 型规划，最后映射回三维空间。

---

### 6. 基于前馈补偿的双空间 PID 控制

项目实现了两类 PID 控制方案：

- **关节空间 PID**：直接控制电机角度 / 绳长误差；
- **笛卡尔空间 PID**：直接控制动平台位姿误差。

同时加入动力学前馈补偿，根据期望轨迹的位置、速度和加速度计算广义力矩与电机转矩补偿，从而提高轨迹跟踪性能。

---

### 7. 模糊 PID 控制

普通 PID 参数固定，难以适应不同轨迹和负载变化。本项目设计了模糊 PID 控制器，以误差 `e` 和误差变化率 `ec` 为输入，输出 `Kp`、`Ki`、`Kd` 的调整量，实现 PID 参数的在线调节。

---

## 运行环境

建议环境：

| 软件 / 工具 | 推荐版本 |
|---|---|
| MATLAB | R2021a 或更高版本 |
| Simulink | 与 MATLAB 版本一致 |
| Robotics Toolbox | 用于串联机械臂相关仿真 |
| C/C++ 编译环境 | 用于组合生成工具，可选 |
| VS Code | 用于代码管理，可选 |

部分 `.slx` 文件和自动生成目录 `slprj/` 可能与 MATLAB 版本有关，如果打开模型时报版本兼容问题，可尝试使用较新版本 MATLAB 或重新生成 Simulink 缓存文件。

---

## 快速开始

### 1. 克隆仓库

```bash
git clone git@github.com:WU-ss/CDPRs_and_serialrobot_system.git
cd CDPRs_and_serialrobot_system
```

### 2. 使用 MATLAB 打开工程目录

在 MATLAB 中将仓库根目录及其子目录加入路径：

```matlab
addpath(genpath(pwd));
```

### 3. 运行单模块测试

可以分别进入对应目录运行测试脚本，例如：

```matlab
% 力可行空间测试
cd('1.力可行空间相关')
Projection_Method_tset

% 运动学测试
cd('../3.运动学相关')
IKandFK

% 索力分布测试
cd('../4.索力分布相关')
Tension_Distribution_Test

% 轨迹规划测试
cd('../6.T型轨迹规划相关')
Path_Planning_plot
```

### 4. 运行 Simulink 控制模型

进入 `Control_Scheme/` 目录，打开对应模型：

```matlab
open_system('Control_Scheme_Cartesian_PID_Simulink.slx')
```

或：

```matlab
open_system('Control_Scheme_Cartesian_FuzzPID_Simulink.slx')
```

然后点击 Simulink 的 **Run** 按钮进行仿真。

---

## 推荐阅读顺序

如果你是第一次查看本项目，建议按以下顺序理解代码：

```text
1. 1.力可行空间相关
2. 4.索力分布相关
3. 3.运动学相关
4. 5.动力学相关
5. 6.T型轨迹规划相关
6. 7.控制部分各模块仿真测试
7. Control_Scheme
```

如果你更关注控制系统，可以直接从：

```text
Control_Scheme/
```

开始查看 Simulink 模型，再根据模块引用回溯到对应函数文件。

---

## 仿真参数示例

项目中使用的典型 CDPR 参数包括：

| 参数 | 示例值 |
|---|---|
| 绳索数量 | 8 |
| 自由度 | 6 |
| 固定框架尺寸 | 15 m × 11 m × 6 m |
| 动平台尺寸 | 1 m × 1 m × 1 m |
| 最小张力 | 10 N |
| 最大张力 | 1000 N |
| 最大绳索速度 | 3.5 m/s |
| 电机等效半径 | 0.0225 m/rad |
| 动平台质量 | 50 kg |

串联机械臂部分包含两类模型：

- 简化两连杆平面机械臂，用于最大负载分析；
- UR3 机械臂模型，用于串联机械臂运动学和动力学仿真。

---

## 结果展示

项目中包含大量 `.fig` 文件，可直接在 MATLAB 中打开查看仿真结果，例如：

- CDPR 力可行空间图；
- 最大负载能力分布图；
- 正 / 逆运动学误差图；
- 索力分布图；
- 三维 T 型轨迹规划曲线；
- PID / 模糊 PID 控制误差曲线；
- 电机角度、动平台位姿、张力和转矩变化曲线。

打开 `.fig` 文件示例：

```matlab
openfig('图1.fig')
```

---

## 注意事项

1. 本项目文件夹名称包含中文和空格，若 MATLAB 路径识别异常，建议使用 `addpath(genpath(pwd))` 或将仓库放置在不含特殊字符的路径下。
2. 部分 Simulink 模型依赖同目录或上级目录中的 `.m` 函数，请确保运行前已添加完整路径。
3. `slprj/` 为 Simulink 自动生成目录，不影响算法理解。
4. `.fig` 文件为仿真结果图，可用于查看历史结果，但重新仿真时可能生成新的结果文件。
5. `Get_all_choice.exe` 为组合生成工具，如在其他平台无法运行，可使用 `Get_all_choice.cpp` 重新编译。
6. 若 Robotics Toolbox 未安装，串联机械臂相关脚本可能无法正常运行，可通过 `RTB.mltbx` 安装。

---

## 适用场景

- CDPR 建模与控制研究；
- 绳索悬挂机械臂系统仿真；
- 大工作空间机器人轨迹规划；
- 多绳索张力分配算法验证；
- 串联机械臂与并联平台组合系统研究；
- MATLAB / Simulink 控制系统课程设计；
- 本科毕设、研究生课题和科研原型开发。

---

## 未来改进方向

后续可以继续扩展：

- 引入绳索弹性、质量和下垂模型；
- 建立更完整的 CDPR + 串联机械臂耦合动力学模型；
- 支持更多自由度机械臂的最大负载分析；
- 使用非线性控制、MPC 或鲁棒控制改进控制性能；
- 加入避障路径规划和三维曲线轨迹生成；
- 与 MuJoCo / ROS / 实机平台进行联动验证；
- 引入视觉感知，实现感知-规划-控制闭环。

---

## 开源说明

本项目仅用于学习、科研和工程原型验证。你可以自由下载、修改和二次开发本项目代码。若将其用于实际机器人系统，请根据真实硬件参数、执行器能力、传感器精度和安全约束进行充分验证。

---

## 联系方式

- GitHub：<https://github.com/WU-ss>
- 仓库地址：<https://github.com/WU-ss/CDPRs_and_serialrobot_system>

如遇到运行错误、模型兼容问题或有改进建议，欢迎提交 Issue。
