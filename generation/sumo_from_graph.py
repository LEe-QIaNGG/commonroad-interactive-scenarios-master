"""
从 CommonRoad 场景生成 SUMO 配置文件（基于图的路由生成）

该工具可以从 CommonRoad 场景文件生成以下配置文件：
1. {scenario_name}.net.xml - SUMO网络文件（通过 converter 生成）
2. {scenario_name}.random.rou.xml - SUMO车辆路由文件（从 net.xml 直接生成）
3. {scenario_name}.sumo.cfg - SUMO配置文件（可选）

使用 commonroad-sumo-interface 包的 CR2SumoMapConverter 来生成 net.xml，
然后直接从 net.xml 中提取连续 edge 路径生成 rou.xml。

作者: Auto-generated
"""

__author__ = "Auto-generated"
__version__ = "0.1"

import os
import pickle
import shutil
import random
import sys
import yaml
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from collections import defaultdict

REPO_ROOT = Path(__file__).resolve().parents[1]
LOCAL_COMMONROAD_SUMO_DIR = REPO_ROOT / "commonroad_sumo"
if LOCAL_COMMONROAD_SUMO_DIR.exists() and str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# 配置文件路径
CONFIG_FILE = REPO_ROOT / "config" / "sumo_from_graph.yaml"

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_sumo.sumo_config.default import DefaultConfig
from commonroad_sumo.cr2sumo import CR2SumoMapConverter, CR2SumoMapConverterConfig

import xml.dom.minidom

# 导入共同工具函数
try:
    from generation.sumo_utils import (
        remove_all_traffic_lights,
        rename_converter_outputs,
        update_sumo_config_file,
    )
except ImportError:
    # 如果绝对导入失败，尝试相对导入
    from .sumo_utils import (
        remove_all_traffic_lights,
        rename_converter_outputs,
        update_sumo_config_file,
    )


def generate_routes_from_net_xml(
    net_file_path: Path,
    output_folder: Path,
    scenario_name: str,
    num_routes: int = 10,
    min_edges: int = 3,
    max_edges: int = 10,
    car_follow_model: str = "Krauss",
    depart_time_start: float = 0.0,
    depart_time_interval: float = 1.5,
    seed: Optional[int] = None,
) -> str:
    """
    直接从 net.xml 中提取连续 edge 路径生成 rou.xml 文件
    
    该方法通过构建有向图，从随机起点开始，沿着 edge 的 from/to 节点连接性
    生成连续的正向路径，确保不会逆行。
    
    :param net_file_path: SUMO 网络文件路径 (.net.xml)
    :param output_folder: 输出文件夹路径
    :param scenario_name: 场景名称
    :param num_routes: 要生成的路径数量上限（如果edge数量超过此值，则随机选择），默认 10
    :param min_edges: 每条路径的最少 edge 数量，默认 3
    :param max_edges: 每条路径的最多 edge 数量，默认 10
    :param car_follow_model: 跟驰模型名称，默认 "Krauss"
    :param depart_time_start: 所有车辆的出发时间（秒），默认 0.0（所有车辆同时出发）
    :param depart_time_interval: 已废弃，保留用于兼容性
    :param seed: 随机种子，用于可重复性
    :return: 生成的 rou.xml 文件名
    """
    if seed is not None:
        random.seed(seed)
    
    # 解析 net.xml 构建有向图
    if not net_file_path.exists():
        raise FileNotFoundError(f"找不到网络文件: {net_file_path}")
    
    try:
        tree = ET.parse(net_file_path)
    except Exception as exc:
        raise ValueError(f"解析网络文件失败 {net_file_path}: {exc}")
    
    root = tree.getroot()
    
    # 构建有向图：edge_id -> {from, to}, node -> [outgoing_edges]
    edge_info: Dict[str, Dict[str, str]] = {}
    edges_from_node: Dict[str, List[str]] = defaultdict(list)
    all_edges: List[str] = []
    
    for edge in root.findall("edge"):
        # 跳过 internal edge（junction 内部的边）
        if edge.attrib.get("function") == "internal":
            continue
        
        edge_id = edge.attrib.get("id")
        if not edge_id:
            continue
        
        from_node = edge.attrib.get("from")
        to_node = edge.attrib.get("to")
        if not from_node or not to_node:
            continue
        
        edge_info[edge_id] = {"from": from_node, "to": to_node}
        edges_from_node[from_node].append(edge_id)
        all_edges.append(edge_id)
    
    if not edge_info:
        raise ValueError("网络中未找到可用的边")
    
    # 解析 connection 元素，构建有效的后继关系
    # 在 SUMO 中，即使两个 edge 的节点相同，也需要有 connection 才能通行
    # 这是 SUMO 网络的标准方式，必须严格遵循
    valid_connections: Dict[str, List[str]] = defaultdict(list)
    for connection in root.findall("connection"):
        from_edge = connection.attrib.get("from")
        to_edge = connection.attrib.get("to")
        # 跳过 internal edge（junction 内部的边）之间的连接
        # 只保留普通 edge 之间的连接
        if from_edge and to_edge and not from_edge.startswith(":") and not to_edge.startswith(":"):
            # 确保目标 edge 存在于我们的 edge 列表中
            if to_edge in all_edges:
                valid_connections[from_edge].append(to_edge)
    
    # 构建后继关系：edge_id -> [successor_edge_ids]
    # 完全依赖 connection 信息，这是 SUMO 网络的标准方式
    successors: Dict[str, List[str]] = {}
    for edge_id in all_edges:
        # 只使用有明确 connection 的后继 edge
        successors[edge_id] = valid_connections.get(edge_id, [])
    
    def generate_path_from_start(start_edge: str, max_length: int = max_edges) -> List[str]:
        """从指定起点edge生成一条连续路径
        
        :param start_edge: 起始edge ID
        :param max_length: 路径的最大长度
        :return: edge ID列表
        """
        path = [start_edge]
        current_edge = start_edge
        
        # 沿着后继 edge 随机游走
        for _ in range(max_length - 1):
            next_edges = successors.get(current_edge, [])
            if not next_edges:
                # 没有后继 edge，路径结束
                break
            
            # 随机选择一个后继 edge
            next_edge = random.choice(next_edges)
            path.append(next_edge)
            current_edge = next_edge
        
        return path
    
    # 先随机选择num_routes个不重复的edge作为起始点
    # 然后为这些edge生成route，确保生成足够数量的有效路径
    num_start_edges = min(num_routes, len(all_edges))
    available_edges = all_edges.copy()
    random.shuffle(available_edges)  # 随机打乱edge列表
    
    routes = []
    used_start_edges = set()
    
    # 尝试生成num_routes条有效路径
    for start_edge in available_edges:
        if len(routes) >= num_routes:
            break
        
        # 跳过已使用的起始edge
        if start_edge in used_start_edges:
            continue
        
        # 从该edge开始生成路径
        path = generate_path_from_start(start_edge, max_edges)
        
        # 只保留满足最小长度要求的路径
        if len(path) >= min_edges:
            routes.append(path)
            used_start_edges.add(start_edge)
    
    if not routes:
        raise ValueError(f"无法生成满足要求的路径（最少 {min_edges} 条 edge）")
    
    # 如果生成的路径数量少于num_routes，输出警告
    if len(routes) < num_routes:
        print(f"  警告: 只生成了 {len(routes)} 条有效路径（要求 {num_routes} 条），可能因为网络中的edge数量不足或路径长度限制")
    
    # 生成 rou.xml 文件
    route_file_path = output_folder / f"{scenario_name}.random.rou.xml"
    with open(route_file_path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n')
        
        # 定义车辆类型
        f.write(f'    <vType id="random_vehicle" accel="2.5" decel="7.5" length="4.5" maxSpeed="35" guiShape="passenger" carFollowModel="{car_follow_model}"/>\n')
        
        # 写入车辆
        # 所有车辆同时出发，使用相同的depart_time_start
        for idx, route in enumerate(routes):
            edges_str = " ".join(route)
            vehicle_id = f"vehicle_{idx}"
            f.write(f'    <vehicle id="{vehicle_id}" type="random_vehicle" depart="{depart_time_start:.2f}" departLane="free" departPos="random">\n')
            f.write(f'        <route edges="{edges_str}"/>\n')
            f.write("    </vehicle>\n")
        
        f.write("</routes>\n")
    
    print(f"  ✓ 已从 net.xml 生成 {len(routes)} 条路径到: {route_file_path.name}")
    print(f"    路径长度范围: {min(len(r) for r in routes)} - {max(len(r) for r in routes)} edges")
    print(f"    所有车辆同时出发（depart={depart_time_start:.2f}秒），起始edge唯一")
    return route_file_path.name


# 共同函数已移至 generation.sumo_utils 模块


def load_config_from_yaml(config_path: Optional[Path] = None) -> Dict:
    """从 YAML 文件加载配置
    
    :param config_path: 配置文件路径，如果为 None 则使用默认路径
    :return: 配置字典
    """
    if config_path is None:
        config_path = CONFIG_FILE
    
    if not config_path.exists():
        print(f"  警告: 配置文件 {config_path} 不存在，使用默认配置")
        return {}
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        return config if config else {}
    except Exception as e:
        print(f"  警告: 读取配置文件 {config_path} 失败: {e}，使用默认配置")
        return {}


def generate_sumo_files_from_commonroad_graph(
    commonroad_scenario_path: str,
    output_folder: str,
    scenario_name: Optional[str] = None,
    converter_config: Optional[CR2SumoMapConverterConfig] = None,
    config_file: Optional[Path] = None,
    **default_config_kwargs
) -> str:
    """
    从 CommonRoad 场景文件生成 SUMO 仿真所需的配置文件（基于图的路由生成）
    
    流程：
    1. 读取 CommonRoad 场景文件 (.cr.xml)
    2. 使用 CR2SumoMapConverter 生成 net.xml 等文件
    3. 从 net.xml 中直接提取连续 edge 路径生成 rou.xml
    
    :param commonroad_scenario_path: CommonRoad场景文件路径 (.cr.xml)
    :param output_folder: 输出文件夹路径
    :param scenario_name: 场景名称（如果不提供，将从文件名提取）
    :param simulation_steps: 仿真步数，默认500
    :param step_length: 每步时间长度（秒），默认0.1
    :param converter_config: CR2SumoMapConverterConfig 配置对象（可选）
    :param num_routes: 要生成的路径数量，默认 10
    :param min_edges: 每条路径的最少 edge 数量，默认 3
    :param max_edges: 每条路径的最多 edge 数量，默认 10
    :param car_follow_model: 跟驰模型名称，默认 "Krauss"
    :param depart_time_start: 第一个车辆的出发时间（秒），默认 0.0
    :param depart_time_interval: 车辆出发时间间隔（秒），默认 1.5
    :param seed: 随机种子，用于可重复性
    :param converter_config: CR2SumoMapConverterConfig 配置对象（可选）
    :param config_file: 配置文件路径，如果提供则从此文件读取所有配置参数
    :param default_config_kwargs: DefaultConfig的其他参数
    :return: 输出文件夹路径
    """
    # 从 YAML 文件加载配置
    yaml_config = load_config_from_yaml(config_file)
    
    # 从 YAML 配置读取所有参数
    simulation_steps = yaml_config.get('simulation', {}).get('steps', 500)
    step_length = yaml_config.get('simulation', {}).get('step_length', 0.1)
    num_routes = yaml_config.get('route_generation', {}).get('num_routes', 10)
    min_edges = yaml_config.get('route_generation', {}).get('min_edges', 3)
    max_edges = yaml_config.get('route_generation', {}).get('max_edges', 10)
    car_follow_model = yaml_config.get('vehicle', {}).get('car_follow_model', 'Krauss')
    depart_time_start = yaml_config.get('departure', {}).get('start', 0.0)
    depart_time_interval = yaml_config.get('departure', {}).get('interval', 1.5)
    seed = yaml_config.get('random_seed')
    generate_cfg = yaml_config.get('options', {}).get('generate_cfg', True)
    
    # 确保输出文件夹存在
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    # 读取 CommonRoad 场景
    print(f"正在读取 CommonRoad 场景: {commonroad_scenario_path}")
    scenario_reader = CommonRoadFileReader(commonroad_scenario_path)
    scenario, planning_problem_set = scenario_reader.open()

    # 确定场景名称
    if scenario_name is None:
        scenario_name = (
            str(scenario.scenario_id) if scenario.scenario_id else Path(commonroad_scenario_path).stem
        )
        if scenario_name.endswith('.cr'):
            scenario_name = scenario_name[:-3]

    # 创建转换器配置
    if converter_config is None:
        converter_config = CR2SumoMapConverterConfig.from_scenario(scenario)
        converter_config.highway_mode = False
    
    converter = CR2SumoMapConverter(scenario, converter_config)

    # 生成 SUMO 文件（net.xml 等）
    print(f"正在生成 SUMO 网络文件...")
    try:
        sumo_project = converter.create_sumo_files(output_folder=output_folder)
    except ValueError as exc:
        print("  警告: 交通灯转换失败，尝试移除交通灯后重试。")
        remove_all_traffic_lights(scenario.lanelet_network)
        converter = CR2SumoMapConverter(scenario, converter_config)
        sumo_project = converter.create_sumo_files(output_folder=output_folder)

    # 重命名转换器输出的文件
    original_project_name = (
        str(scenario.scenario_id) if scenario.scenario_id else scenario_name
    )
    rename_converter_outputs(output_folder, original_project_name, scenario_name)

    # 从 net.xml 生成随机路由文件
    net_file_path = output_folder / f"{scenario_name}.net.xml"
    print(f"正在从 net.xml 生成随机路由文件...")
    route_file_name = generate_routes_from_net_xml(
        net_file_path=net_file_path,
        output_folder=output_folder,
        scenario_name=scenario_name,
        num_routes=num_routes,
        min_edges=min_edges,
        max_edges=max_edges,
        car_follow_model=car_follow_model,
        depart_time_start=depart_time_start,
        depart_time_interval=depart_time_interval,
        seed=seed,
    )

    # 复制 CommonRoad 场景文件到输出文件夹
    output_cr_path = output_folder / f"{scenario_name}.cr.xml"
    if Path(commonroad_scenario_path).resolve() != output_cr_path.resolve():
        shutil.copy2(commonroad_scenario_path, output_cr_path)

    # 生成 SUMO 配置文件
    if generate_cfg:
        print(f"正在生成 SUMO 配置文件...")
        sumo_cfg_path = output_folder / f"{scenario_name}.sumo.cfg"
        update_sumo_config_file(
            sumo_cfg_path=sumo_cfg_path,
            scenario_name=scenario_name,
            route_file_name=route_file_name,
            step_length=step_length,
        )

    # 创建并保存 DefaultConfig
    conf = DefaultConfig()
    conf.scenario_name = scenario_name
    conf.simulation_steps = simulation_steps
    conf.step_length = step_length
    
    # 设置其他配置参数
    for key, value in default_config_kwargs.items():
        if hasattr(conf, key):
            setattr(conf, key, value)

    # 保存仿真配置
    config_pickle_path = output_folder / "simulation_config.p"
    with open(config_pickle_path, "wb") as f:
        pickle.dump(conf, f)

    print(f"✓ 成功生成配置文件到: {output_folder}")
    return str(output_folder)


if __name__ == "__main__":
    import argparse

    # 加载默认配置
    default_config = load_config_from_yaml()

    def _default_paths_from_name(scenario_name: str):
        """根据场景名称推断默认路径"""
        repo_root = Path(__file__).resolve().parents[1]
        scenario_folder = repo_root / "interactive_scenarios" / scenario_name
        commonroad_file = scenario_folder / f"{scenario_name}.cr.xml"
        if not commonroad_file.exists():
            candidates = sorted(scenario_folder.glob("*.cr.xml"))
            if len(candidates) == 1:
                commonroad_file = candidates[0]
            elif len(candidates) == 0:
                raise FileNotFoundError(
                    f"未在 {scenario_folder} 找到任何 .cr.xml 文件，请使用 --commonroad_file 指定。"
                )
            else:
                names = ", ".join(str(path.name) for path in candidates)
                raise ValueError(
                    f"{scenario_folder} 中存在多个 .cr.xml 文件 ({names})，请使用 --commonroad_file 指定目标文件。"
                )
        return commonroad_file, scenario_folder

    parser = argparse.ArgumentParser(
        description="从 CommonRoad 场景生成 SUMO 配置文件（基于图的路由生成）"
    )
    parser.add_argument(
        "--commonroad_file",
        type=str,
        help="CommonRoad场景文件路径 (.cr.xml)"
    )
    parser.add_argument(
        "--output_folder",
        type=str,
        help="输出文件夹路径"
    )
    parser.add_argument(
        "--scenario_name",
        type=str,
        default=None,
        help="场景名称（默认从文件名提取）"
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help=f"配置文件路径（默认: {CONFIG_FILE}）"
    )
    
    args = parser.parse_args()

    # 加载配置文件
    config_file = Path(args.config) if args.config else None

    commonroad_file = args.commonroad_file
    output_folder = args.output_folder

    # 如果提供了场景名称但没有提供文件路径，尝试推断
    if args.scenario_name and (not commonroad_file or not output_folder):
        default_cr, default_folder = _default_paths_from_name(args.scenario_name)
        commonroad_file = commonroad_file or str(default_cr)
        output_folder = output_folder or str(default_folder)

    if not commonroad_file:
        raise ValueError("必须提供 --commonroad_file，或通过 --scenario_name 推断。")
    if not output_folder:
        raise ValueError("必须提供 --output_folder，或通过 --scenario_name 推断。")

    # 所有参数都从配置文件读取
    generate_sumo_files_from_commonroad_graph(
        commonroad_scenario_path=commonroad_file,
        output_folder=output_folder,
        scenario_name=args.scenario_name,
        config_file=config_file,
    )