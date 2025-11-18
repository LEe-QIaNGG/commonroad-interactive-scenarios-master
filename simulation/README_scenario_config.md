# CommonRoad场景到SUMO仿真配置文件生成指南

## 概述

要在自定义的CommonRoad场景上进行SUMO仿真，需要生成以下配置文件。本工具使用 **commonroad-sumo-interface** 包的 `CR2SumoMapConverter` 来自动从CommonRoad场景文件生成这些配置。

**核心转换类**: `CR2SumoMapConverter` - 负责将 CommonRoad 的 lanelet 网络转换为 SUMO 的网络格式。

## 需要的配置文件

根据 `simulations.py` 的分析，进行SUMO仿真需要以下文件：

### 1. CommonRoad场景文件
- **文件名**: `{scenario_name}.cr.xml`
- **说明**: 你的CommonRoad场景文件
- **来源**: 用户提供

### 2. SUMO网络文件
- **文件名**: `{scenario_name}.net.xml`
- **说明**: SUMO网络拓扑文件，描述道路网络结构
- **来源**: 从CommonRoad场景自动生成（通过 `CR2SumoMapConverter.create_sumo_files()`）

### 3. SUMO车辆路由文件
- **文件名**: `{scenario_name}.vehicle.rou.xml` 或 `{scenario_name}.rou.xml`
- **说明**: SUMO车辆路由文件，描述车辆的路由和出发时间
- **来源**: 从CommonRoad场景自动生成（通过 `CR2SumoMapConverter.create_sumo_files()`）

### 4. SUMO配置文件
- **文件名**: `{scenario_name}.sumo.cfg`
- **说明**: SUMO仿真主配置文件，指定网络文件、路由文件等
- **来源**: 自动生成（通过 `CR2SumoMapConverter.create_sumo_files()` 或 `scenario_config_generator.py`）

### 5. SUMO附加文件（可选）
- **文件名**: `{scenario_name}.add.xml`
- **说明**: 包含交通信号灯等附加元素
- **来源**: 从CommonRoad场景自动生成（通过 `CR2SumoMapConverter.create_sumo_files()`）

### 6. 仿真配置
- **文件名**: `simulation_config.p`
- **说明**: Python pickle格式的 `DefaultConfig` 对象，包含仿真参数
- **来源**: 自动生成（通过 `scenario_config_generator.py`）

## 使用方法

### 方法1: 使用Python脚本生成所有文件

```python
from simulation.scenario_config_generator import generate_sumo_files_from_commonroad

# 从CommonRoad场景生成所有配置文件
output_path = generate_sumo_files_from_commonroad(
    commonroad_scenario_path="path/to/your/scenario.cr.xml",
    output_folder="./my_scenario",
    scenario_name="my_scenario",
    simulation_steps=500,  # 仿真步数
    step_length=0.1  # 每步时间长度（秒）
)
```

### 方法2: 使用命令行工具

```bash
python -m simulation.scenario_config_generator \
    path/to/your/scenario.cr.xml \
    ./my_scenario \
    --scenario-name my_scenario \
    --simulation-steps 500 \
    --step-length 0.1
```

### 方法3: 如果已有SUMO文件，只生成配置

如果你已经有 `.net.xml` 和 `.vehicle.rou.xml` 文件，只需要生成配置：

```python
from simulation.scenario_config_generator import generate_config_for_existing_scenario

output_path = generate_config_for_existing_scenario(
    commonroad_scenario_path="path/to/your/scenario.cr.xml",
    output_folder="./my_scenario",
    scenario_name="my_scenario",
    simulation_steps=500,
    step_length=0.1
)
```

## 配置文件结构说明

### SUMO配置文件 (.sumo.cfg)

```xml
<?xml version="1.0" ?>
<configuration>
    <input>
        <net-file value="{scenario_name}.net.xml"/>
        <route-files value="{scenario_name}.vehicle.rou.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <step-length value="0.1"/>
    </time>
    <!-- 其他配置... -->
</configuration>
```

### 仿真配置 (simulation_config.p)

包含 `DefaultConfig` 对象，主要属性：
- `scenario_name`: 场景名称
- `simulation_steps`: 仿真步数
- `step_length`: 每步时间长度

## 完整使用示例

```python
import os
from simulation.scenario_config_generator import generate_sumo_files_from_commonroad
from simulation.simulations import simulate_without_ego

# 1. 生成配置文件
output_folder = generate_sumo_files_from_commonroad(
    commonroad_scenario_path="my_scenario.cr.xml",
    output_folder="./my_scenario_output",
    scenario_name="my_scenario",
    simulation_steps=500,
    step_length=0.1
)

# 2. 运行仿真
scenario_without_ego, pps = simulate_without_ego(
    interactive_scenario_path=output_folder,
    output_folder_path="./outputs/videos",
    create_video=False
)
```

## 注意事项

1. **场景文件要求**: CommonRoad场景文件必须包含有效的道路网络和动态障碍物信息
2. **输出文件夹**: 所有生成的文件将保存在指定的输出文件夹中
3. **文件命名**: 确保场景名称与文件名一致
4. **依赖**: 
   - 优先使用 `commonroad-sumo-interface` 包（推荐）
   - 如果未安装，会自动回退到使用 `sumocr` 和 `commonroad` 库
5. **API兼容性**: 代码会自动尝试不同的参数组合以兼容不同版本的 `commonroad-sumo-interface`

## 故障排除

### 问题1: `FileNotFoundError: simulation_config.p`
- **原因**: 配置文件未生成
- **解决**: 运行 `generate_sumo_files_from_commonroad()` 生成配置文件

### 问题2: SUMO文件生成失败
- **原因**: CommonRoad场景可能不包含有效的网络信息
- **解决**: 检查场景文件是否包含有效的车道网络

### 问题3: 场景名称不匹配
- **原因**: `simulation_config.p` 中的 `scenario_name` 与文件名不一致
- **解决**: 确保使用相同的场景名称

## 相关文件

- `simulation/scenario_config_generator.py`: 配置文件生成工具
- `simulation/simulations.py`: 仿真函数
- `example_generate_config.py`: 使用示例

