"""
示例：从自定义CommonRoad场景生成SUMO仿真配置文件

使用方法：
1. 准备你的CommonRoad场景文件 (.cr.xml)
2. 运行此脚本生成所需的配置文件
3. 使用生成的配置文件进行仿真
"""

import os
import sys
from pathlib import Path

# 添加项目路径
sys.path.append(os.path.join(os.path.dirname(__file__)))

from simulation.scenario_config_generator import (
    generate_sumo_files_from_commonroad
)


def example_generate_from_commonroad():
    """
    示例：从CommonRoad场景文件生成所有SUMO配置文件
    """
    # 指定你的CommonRoad场景文件路径
    commonroad_file = "interactive_scenarios/CHN_TST-2_1_T-23/CHN_TST-2_1_T-23.cr.xml"
    # 指定输出文件夹
    output_folder = "interactive_scenarios/CHN_TST-2_1_T-23"
    
    # 场景名称（如果不指定，将从文件名提取）
    scenario_name = "CHN_TST-2_1_T-23"
    solution_file = "interactive_scenarios/CHN_TST-2_1_T-23/CHN_TST-2_1_T-23_solution.xml"
    
    # 生成配置文件
    try:
        output_path = generate_sumo_files_from_commonroad(
            commonroad_scenario_path=commonroad_file,
            output_folder=output_folder,
            scenario_name=scenario_name,
            simulation_steps=500,  # 仿真步数
            step_length=0.1,  # 每步时间长度（秒）
            solution_file=solution_file,
        )
        print(f"\n✓ 配置文件已生成到: {output_path}")

    except Exception as e:
        print(f"✗ 生成配置文件时出错: {e}")
        import traceback
        traceback.print_exc()


def example_generate_config_only():
    """
    示例：如果已有SUMO文件，只生成配置
    """
    # 如果你的场景已经有.net.xml和.vehicle.rou.xml文件
    commonroad_file = "path/to/your/scenario.cr.xml"
    output_folder = "./my_custom_scenario"
    scenario_name = "my_custom_scenario"
    
    try:
        output_path = generate_config_for_existing_scenario(
            commonroad_scenario_path=commonroad_file,
            output_folder=output_folder,
            scenario_name=scenario_name,
            simulation_steps=500,
            step_length=0.1
        )
        print(f"\n✓ 配置文件已生成到: {output_path}")
    except Exception as e:
        print(f"✗ 生成配置文件时出错: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    example_generate_from_commonroad()
    # example_generate_config_only()

