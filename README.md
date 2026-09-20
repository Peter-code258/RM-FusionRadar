# RM-FusionRadar

基于主流开源方案的 RoboMaster 雷达系统研究性项目 —— C++ 五大子系统实现。

本仓库以《RoboMaster雷达系统研究性项目设计与开发文档》的技术路线为依据，
将五大技术模块分别实现为可独立编译运行的 C++ 系统，并以独立分支管理。

## 五大子系统分支

| 分支名 | 子系统 | 核心定位 |
|--------|--------|----------|
| `lidar-perception` | 激光雷达点云感知子系统 | 核心 3D 感知，抗环境干扰 |
| `sensor-fusion` | 多传感器融合处理子系统 | 系统中枢，多源数据整合 |
| `sdr-perception` | SDR 软件无线电感知子系统 | 官方数据补全，零误差定位 |
| `monocular-radar` | 低成本单目视觉雷达子系统 | 低成本替代，无激光雷达方案 |
| `uav-countermeasure` | 无人机跟踪与反制子系统 | 空中目标防御，进阶功能 |

## 目录结构（公共部分）

```
common/
  include/
    common_types.h     # 统一目标数据结构
    math_utils.h       # 卡尔曼滤波、匈牙利匹配、RANSAC、矩阵运算
    config_loader.h    # 简易配置加载
    logger.h           # 分级日志
```

## 构建

每个子系统分支均为独立的 CMake 工程：

```bash
git checkout <branch-name>
mkdir build && cd build
cmake .. && make -j
```

## 设计目标

- 模块化、解耦、可降级
- 兼容入门级 / 进阶级 / 高端级硬件
- 赛场鲁棒性优先
