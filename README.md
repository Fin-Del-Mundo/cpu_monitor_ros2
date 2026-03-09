# CPU Monitor ROS2 Package

实时监测X86 CPU核心温度，按1秒周期发布。**只统计进程亲和性允许且使用率≥5%的CPU核心**，确保发布数据准确客观。

## 快速开始

### 构建
```bash
colcon build --packages-select cpu_monitor
```

### 运行
```bash
source install/setup.bash
ros2 run cpu_monitor cpu_monitor_node

# 启用调试日志
ros2 run cpu_monitor cpu_monitor_node --ros-args --log-level DEBUG
```

### 查看数据
```bash
ros2 topic echo /x86_temperature          # 实时温度
ros2 topic hz /x86_temperature            # 发布频率
```

## 核心特性

| 特性 | 说明 |
|------|------|
| 1秒周期发布 | 固定1秒发送一次温度数据 |
| **CPU亲和性检查** | 从 `/proc/self/status` 读取 `Cpus_allowed_list`，只计算允许的CPU |
| 使用率过滤 | 只统计使用率≥5%的核心（排除低负载核心） |
| 多源温度 | 优先硬件传感器，备选CPU频率推算 |
| 可配置阈值 | 默认5%，可根据需求调整（0%-100%） |
| float32精度 | 发布浮点温度数据，精度0.1°C |

## 消息定义

**Topic**: `/x86_temperature`  
**消息类型**: `cpu_monitor::msg::X86CpuTemperature`

```
float32 x86_temperature    # 平均温度，单位：°C
```

## 核心算法

### 1. CPU亲和性检查

从 `/proc/self/status` 读取进程允许的CPU列表：

```bash
cat /proc/self/status | grep Cpus_allowed_list
# 输出示例：
# Cpus_allowed_list:	0-15       # 允许使用CPU 0-15
# Cpus_allowed_list:	0,2,4,6    # 只允许使用 0,2,4,6
```

**支持格式**：
- 范围: `0-max`
- 单个: `0,2,4,6`
- 混合: `0-3,8-11`

**工作原理**：
- 系统启动时读取并缓存允许的CPU列表
- 只有在允许列表中的CPU才会被计入温度统计
- 自动适应GRUB锁核、进程绑核等场景

### 2. CPU使用率计算

从 `/proc/stat` 读取CPU时间统计，计算使用率：

$$\text{使用率} = \frac{\text{总时间变化} - \text{空闲时间变化}}{\text{总时间变化}} \times 100\%$$

**时间分量**：
- 非空闲 = user + nice + system + irq + softirq
- 空闲 = idle + iowait
- 总时间 = 非空闲 + 空闲

**精度**：1秒采样周期，每次计算与前一秒的差值

### 3. 温度数据过滤

**核心有效性判断**：
```
活跃 = (CPU在亲和列表中) AND (使用率 ≥ 5%)
```

**温度来源优先级**：
1. `/sys/class/thermal/thermal_zoneX/temp` (硬件传感器) → 准确度最高
2. `/proc/cpuinfo` cpu MHz (频率推算) → 备选方案

**最终计算**：
$$T_{avg} = \frac{\sum_{i \in \text{活跃}} T_i}{n}$$

其中 $n$ = 活跃核心数

## 配置参数

**参数名**: `cpu_usage_threshold_`  
**位置**: `src/cpu_monitor_node.cpp` 
**默认值**: `5.0` (百分比)

### 推荐配置

| 阈值 | 模式 | 说明 |
|------|------|------|
| 0.0% | 完全 | 包含所有允许的CPU（无过滤） |
| 1.0% | 宽松 | 包含轻微负载核心 |
| 5.0% | 标准 | **推荐**，平衡准确性和响应性 |
| 10.0% | 严格 | 仅统计高负载核心 |

### 数据源优先级

| 优先级 | 数据源 | 用途 |
|--------|--------|------|
| 1 | `/proc/self/status` | 读取进程CPU亲和性列表 |
| 2 | `/proc/stat` | 计算CPU使用率，过滤低使用率核心 |
| 3 | `/sys/class/thermal/thermal_zoneX/temp` | 读取硬件温度（毫度→摄氏度） |
| 4 | `/proc/cpuinfo` cpu MHz | 频率推算温度（备选） |

### 性能特性

- **CPU占用**：极小（每秒一次I/O操作）
- **内存占用**：固定（仅保存两个CPU统计快照和亲和性集合）
- **时序精度**：1秒周期，±100ms误差范围
- **编译**：零警告零错误

## 故障排除

### 温度读取为0

**原因**：无可用的温度传感器  
**检查**：
```bash
ls /sys/class/thermal/              # 检查thermal_zone
cat /proc/cpuinfo | grep "cpu MHz"  # 检查CPU频率
```

### 所有CPU都被过滤

**原因**：系统闲置，所有允许的核心使用率<5%  
**解决**：运行CPU密集任务或降低阈值到1.0%

### CPU亲和性显示为空

**原因**：无法读取 `/proc/self/status`  
**解决**：
```bash
cat /proc/self/status | grep Cpus_allowed_list
# 检查输出并验证权限
```

**降级行为**：无法读取时，自动允许所有在线CPU

### Topic无数据

**原因**：节点未正确启动  
**解决**：
```bash
ros2 run cpu_monitor cpu_monitor_node --ros-args --log-level DEBUG
```

查看日志输出并检查错误信息，应看到：
```
[INFO] CPU Monitor Node initialized (usage threshold: 5.0%)
[INFO] CPU affinity: 16 cores allowed
[INFO] Published CPU temperature: XX.X°C
```

## 系统要求

- ROS 2 Humble 或更新版本
- GCC/Clang C++17 编译器
- Linux 系统（支持/proc和/sys文件系统）
- 硬件温度传感器（可选，无传感器时自动降级）

## 依赖项

- `rclcpp` - ROS2 C++客户端库
- `rosidl_default_generators` - 消息生成工具
- `<set>` - C++标准库（CPU亲和性集合）

## 文件结构

```
cpu_monitor/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包元数据
├── README.md               # 本文件
├── msg/
│   └── X86CpuTemperature.msg   # 消息定义（float32 x86_temperature）
├── src/
│   └── cpu_monitor_node.cpp    # 主程序
```

## 输出示例

```
[INFO] CPU Monitor Node initialized (usage threshold: 5.0%)
[INFO] CPU affinity: 16 cores allowed
[INFO] Published CPU temperature: 52.3°C
[DEBUG] Skipping low-usage CPU cpu2 (1.2%)
[DEBUG] Skipping CPU not in affinity list: cpu15
[DEBUG] Calculated average temperature from 4 cores: 52.3°C
```

## 使用场景

| 场景 | 配置 | 说明 |
|------|------|------|
| 服务器热管理 | 10.0% | 只关注高负载核心温度 |
| 通用系统监测 | 5.0% | **推荐** |
| 绑核应用热监控 | 5.0% | 自动适应绑核，监测活跃核心 |
| 嵌入式系统 | 1.0% | 低核心数，需全部统计 |
| 基准测试 | 0.0% | 完整数据用于研究 |

## 完整流程图

```
启动
  ↓
初始化CPU统计
  ↓
读取CPU亲和性列表 (/proc/self/status)
  ↓
每1秒执行：
  1. 更新前一次CPU统计
  2. 读取新的/proc/stat
  3. 计算每个核心使用率
  4. 遍历所有thermal_zone
  5. 检查：在亲和列表中? ✓ 使用率≥5%? ✓
  6. 收集符合条件的温度
  7. 计算平均值 (float32精度)
  8. 发布X86CpuTemperature消息到 /x86_temperature
```

## 许可证

Apache 2.0