# SDR 软件无线电感知子系统 (feature/sdr-radio-decoder)

官方信息波解码模块，完成基带信号的协议解析与数据输出。信号前端对接 GNU Radio，核心解析逻辑纯 C++ 实现。

## 技术栈
- C++17、原生 UDP Socket、CRC16、GNU Radio（信号前端）

## 核心功能
- **帧同步**：搜索同步字 `0xAA55`，支持多重同步字（含反相 `0x55AA`）校验，抗干扰定位帧起始
- **CRC-16 校验**：校验失败直接丢弃错误帧，保证数据正确性
- **协议解包**：机器人 ID、x/y 坐标、血量、弹量、状态字
- **卡尔曼轨迹插值**：对 10Hz 官方数据做卡尔曼滤波插值，输出 30Hz 平滑轨迹
- **字节序兼容**：支持大小端配置，适配不同硬件平台
- **丢包容错**：连续丢包时维持轨迹预测，坐标不跳变
- **自动信道扫描**：赛前自动扫描 16 个信道，锁定信号最强的官方工作信道
- **UDP 数据发布**：10Hz（插值后 30Hz）向融合中心发布解析数据

## 编译运行
```bash
mkdir build && cd build
cmake .. && make -j
./sdr_decoder ../config/sdr_config.yaml
```

## 工具
- `channel_scan`：信道扫描与自动锁定
- `sdr_replay <file.bin>`：离线录制数据回放调试

## 运行测试
```bash
cd build && make -j && ./test_sdr
```

## 配置文件
`config/sdr_config.yaml`：字节序、CRC 校验、UDP 地址端口、插值率、信道扫描参数。
