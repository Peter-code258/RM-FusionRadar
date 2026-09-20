# RM-FusionRadar — RoboMaster 雷达系统研究性项目

基于主流开源方案的 RoboMaster 雷达系统研究性项目，以 **C++17** 为核心开发语言，将五大技术模块实现为可独立编译运行的 C++ 子系统，通过统一数据格式与 UDP 通信协议解耦，最终可集成到主系统。

## 系统架构

```
                    ┌─────────────────────────────────────┐
                    │         多传感器融合处理中心           │
                    │  (feature/multi-sensor-fusion)       │
                    │  坐标转换 · 时间同步 · 数据关联 · 融合 │
                    └──────────────┬──────────────────────┘
                                   │ UDP 统一目标列表
           ┌───────────┬───────────┼───────────┬───────────┐
           ▼           ▼           ▼           ▼           ▼
   ┌────────────┐ ┌──────────┐ ┌─────────┐ ┌──────────┐ ┌──────────┐
   │ 激光雷达   │ │ SDR      │ │ 单目视觉 │ │ 无人机    │ │ 作战单位  │
   │ 点云感知   │ │ 无线电   │ │ 雷达     │ │ 跟踪反制 │ │ (输出)    │
   └────────────┘ └──────────┘ └─────────┘ └──────────┘ └──────────┘
```

## 五大子系统分支

| 分支 | 子系统 | 核心定位 | 依赖 |
|------|--------|----------|------|
| `feature/lidar-pointcloud-perception` | 激光雷达点云感知 | 核心 3D 感知，抗环境干扰 | PCL, Eigen3 |
| `feature/multi-sensor-fusion` | 多传感器融合处理中心 | 系统中枢，多源数据整合 | Eigen3 |
| `feature/sdr-radio-decoder` | SDR 软件无线电感知 | 官方数据补全，零误差定位 | 无外部依赖 |
| `feature/mono-vision-radar` | 低成本单目视觉雷达 | 低成本替代，无激光雷达方案 | OpenCV4, Eigen3 |
| `feature/anti-drone-tracking` | 无人机跟踪与反制 | 空中目标防御，进阶功能 | OpenCV4, Eigen3 |

## 公共库 (`common/`)

所有子系统共享的基础模块，位于 `common/include/`：

| 文件 | 功能 |
|------|------|
| `common_types.h` | 统一目标数据结构（`Target`、`Vec3`、`TargetType`、`SensorSource`） |
| `math_utils.h` | 矩阵运算、卡尔曼滤波、匈牙利匹配、RANSAC 平面拟合 |
| `config_loader.h` | 轻量级 YAML 风格配置加载器 |
| `logger.h` | 分级日志（DEBUG/INFO/WARN/ERROR） |

## 统一数据格式

所有子系统输出的目标均遵循 `rm_radar::Target` 结构：

```cpp
struct Target {
    uint32_t id;                 // 唯一目标 ID
    TargetType type;             // 目标类别（步兵/英雄/工程/哨兵/无人机）
    Vec3 position;               // 世界坐标 (x, y, z) 米
    Vec3 velocity;               // 速度向量 (m/s)
    float confidence;            // 置信度 [0, 1]
    uint64_t timestamp_us;       // 硬件时间戳（微秒）
    uint32_t point_count;        // 点数 / 像素数
    float bbox[6];               // 3D 包围盒 [x_min,y_min,z_min,x_max,y_max,z_max]
};
```

## UDP 通信协议

子系统间通过 UDP 传输二进制目标数据：

- **输入**（传感器 → 融合中心）：`[source(1)][count(4)][Target...]`
- **输出**（融合中心 → 作战单位）：`[count(4)][Target...]`

默认端口：
- 激光雷达：8800
- SDR：8801
- 单目视觉：8802
- 无人机反制：8803
- 融合中心输入：8800 / 输出：8900

## 分支开发与集成流程

```
feature/* (独立开发 + 单元测试)
       │
       ▼
   develop (集成测试)
       │
       ▼
   main (最终发布)
```

每个子系统分支均为独立的 CMake 工程，可单独编译运行与测试。

## 编译运行

切换到对应分支后：

```bash
git checkout feature/<subsystem>
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
./<executable> ../config/<config>.yaml
```

运行单元测试：

```bash
cd build && make -j && ./test_<name>
```

## 设计目标

- **模块化、解耦、可降级**：后融合架构，传感器级独立，故障隔离
- **兼容多档硬件**：从入门级单目到高端激光雷达 + SDR
- **赛场鲁棒性优先**：卡尔曼跟踪、降级容错、盲区预测
- **统一接口**：标准化 `Target` 格式 + UDP 协议，子系统可插拔
