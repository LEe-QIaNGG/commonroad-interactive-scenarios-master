"""
从CommonRoad场景生成SUMO仿真所需的配置文件

该工具可以从CommonRoad场景文件生成以下配置文件：
1. {scenario_name}.net.xml - SUMO网络文件
2. {scenario_name}.vehicle.rou.xml - SUMO车辆路由文件
3. {scenario_name}.sumo.cfg - SUMO配置文件
4. simulation_config.p - 仿真配置（pickle格式）

使用 commonroad-sumo-interface 包的 CR2SumoMapConverter 来生成SUMO文件。

作者: Auto-generated
"""

__author__ = "Auto-generated"
__version__ = "0.1"

import os
import pickle
import shutil
from collections import defaultdict, deque
from typing import Optional, Dict, List, Tuple
from pathlib import Path
import random
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
LOCAL_SUMOCR_DIR = REPO_ROOT / "sumocr"
if LOCAL_SUMOCR_DIR.exists() and str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import CommonRoadSolutionReader
from sumocr.sumo_config.default import DefaultConfig
from commonroad_sumo.cr2sumo import CR2SumoMapConverter, CR2SumoMapConverterConfig
from commonroad_sumo.sumolib.net import Edge

import xml.etree.ElementTree as ET


def generate_sumo_files_from_commonroad(
    commonroad_scenario_path: str,
    output_folder: str,
    scenario_name: Optional[str] = None,
    simulation_steps: int = 500,
    step_length: float = 0.1,
    converter_config: Optional[CR2SumoMapConverterConfig] = None,
    solution_file: Optional[str] = None,
    car_follow_model: str = "Krauss",
    num_random_pedestrians: int = 0,
    pedestrian_speed: float = 1.3,
    pedestrian_depart_interval: float = 2.0,
    pedestrian_walk_min: int = 3,
    pedestrian_walk_max: int = 6,
    pedestrian_seed: Optional[int] = None,
    **default_config_kwargs
) -> str:
    """
    从CommonRoad场景文件生成SUMO仿真所需的所有配置文件

    使用 commonroad-sumo-interface 包的 CR2SumoMapConverter 来生成SUMO文件。

    :param commonroad_scenario_path: CommonRoad场景文件路径 (.cr.xml)
    :param output_folder: 输出文件夹路径
    :param scenario_name: 场景名称（如果不提供，将从文件名提取）
    :param simulation_steps: 仿真步数，默认500
    :param step_length: 每步时间长度（秒），默认0.1
    :param converter_config: CR2SumoMapConverterConfig 配置对象（可选）
    :param car_follow_model: 跟驰模型名称，可选值：Krauss, IDM, IDMM, W99, PWagner2009, BKerner, Gipps, Linear, Newell 等，默认 "Krauss"
    :param default_config_kwargs: DefaultConfig的其他参数
    :param num_random_pedestrians: 从 solution 中分配为行人的数量，默认 0（不生成行人）
    :param pedestrian_speed: 行人行走速度（m/s）
    :param pedestrian_depart_interval: 行人出发时间间隔（秒，已废弃，保留用于兼容性）
    :param pedestrian_walk_min: 随机游走的最短边数（已废弃，保留用于兼容性）
    :param pedestrian_walk_max: 随机游走的最长边数（已废弃，保留用于兼容性）
    :param pedestrian_seed: 随机种子（已废弃，保留用于兼容性）
    :return: 输出文件夹路径
    """
    # 确保输出文件夹存在
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    scenario_reader = CommonRoadFileReader(commonroad_scenario_path)
    scenario, planning_problem_set = scenario_reader.open()

    if scenario_name is None:
        scenario_name = (
            str(scenario.scenario_id) if scenario.scenario_id else Path(commonroad_scenario_path).stem
        )
        if scenario_name.endswith('.cr'):
            scenario_name = scenario_name[:-3]

    if converter_config is None:
        converter_config = CR2SumoMapConverterConfig.from_scenario(scenario)
        converter_config.highway_mode = False
    converter = CR2SumoMapConverter(scenario, converter_config)

    # 调用 create_sumo_files() 生成SUMO文件
    # 该方法返回一个 sumo_project 对象
    # 传入 output_folder，确保所有文件输出到目标目录
    try:
        sumo_project = converter.create_sumo_files(output_folder=output_folder)
    except ValueError as exc:
        print("  警告: 交通灯转换失败，尝试移除交通灯后重试。")
        _remove_all_traffic_lights(scenario.lanelet_network)
        converter = CR2SumoMapConverter(scenario, converter_config)
        sumo_project = converter.create_sumo_files(output_folder=output_folder)

    original_project_name = (
        str(scenario.scenario_id) if scenario.scenario_id else scenario_name
    )
    _rename_converter_outputs(output_folder, original_project_name, scenario_name)

    net_file_path = output_folder / f"{scenario_name}.net.xml"
    _ensure_pedestrian_access_on_net(net_file_path)

    generated_route = _generate_routes_from_solution(
            solution_file,
            scenario,
            converter,
            planning_problem_set,
            output_folder,
            scenario_name,
            car_follow_model=car_follow_model,
            num_pedestrians=num_random_pedestrians,
            pedestrian_speed=pedestrian_speed,
        )
    route_file_name = generated_route

    # 复制CommonRoad场景文件到输出文件夹
    output_cr_path = output_folder / f"{scenario_name}.cr.xml"
    if Path(commonroad_scenario_path).resolve() != output_cr_path.resolve():
        shutil.copy2(commonroad_scenario_path, output_cr_path)

    # 检查并重命名文件以匹配预期格式
    _rename_sumo_files_if_needed(output_folder, scenario_name)

    # 更新 SUMO 配置文件
    sumo_cfg_path = output_folder / f"{scenario_name}.sumo.cfg"
    _update_sumo_config_file(str(sumo_cfg_path), scenario_name, route_file_name, step_length)

    # 创建DefaultConfig并保存
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


def _rename_sumo_files_if_needed(output_folder: Path, scenario_name: str):
    """检查并重命名SUMO文件以匹配预期格式"""
    # commonroad-sumo-interface 可能生成 .rou.xml 而不是 .vehicle.rou.xml
    rou_file = output_folder / f"{scenario_name}.rou.xml"
    vehicle_rou_file = output_folder / f"{scenario_name}.vehicle.rou.xml"
    
    if rou_file.exists() and not vehicle_rou_file.exists():
        shutil.copy2(rou_file, vehicle_rou_file)
        print(f"  已复制 {rou_file.name} 为 {vehicle_rou_file.name}")


def _update_sumo_config_file(
    sumo_cfg_path: str,
    scenario_name: str,
    route_file_name: str,
    step_length: float,
):
    """更新SUMO配置文件以确保引用正确的网络和路由文件，并添加必要的processing配置（包括collision设置）"""
    tree = ET.parse(sumo_cfg_path)
    root = tree.getroot()


    input_element = root.find("input")
    if input_element is None:
        input_element = ET.SubElement(root, "input")

    net_file_element = input_element.find("net-file")
    if net_file_element is None:
        net_file_element = ET.SubElement(input_element, "net-file")
    net_file_element.set("value", f"{scenario_name}.net.xml")

    route_files_element = input_element.find("route-files")
    if route_files_element is None:
        route_files_element = ET.SubElement(input_element, "route-files")
    route_files_element.set("value", route_file_name)

    time_element = root.find("time")
    if time_element is None:
        time_element = ET.SubElement(root, "time")

    step_length_element = time_element.find("step-length")
    if step_length_element is None:
        step_length_element = ET.SubElement(time_element, "step-length")
    step_length_element.set("value", str(step_length))

    # 添加或更新 processing 部分，包括 collision 配置
    processing_element = root.find("processing")
    if processing_element is None:
        processing_element = ET.SubElement(root, "processing")

    # 设置 lateral-resolution
    lateral_resolution = processing_element.find("lateral-resolution")
    if lateral_resolution is None:
        lateral_resolution = ET.SubElement(processing_element, "lateral-resolution")
    lateral_resolution.set("value", "1.0")

    # 设置 max-depart-delay
    max_depart_delay = processing_element.find("max-depart-delay")
    if max_depart_delay is None:
        max_depart_delay = ET.SubElement(processing_element, "max-depart-delay")
    max_depart_delay.set("value", "5")

    # 设置 time-to-teleport
    time_to_teleport = processing_element.find("time-to-teleport")
    if time_to_teleport is None:
        time_to_teleport = ET.SubElement(processing_element, "time-to-teleport")
    time_to_teleport.set("value", "-1")

    # 设置 collision.mingap-factor
    collision_mingap = processing_element.find("collision.mingap-factor")
    if collision_mingap is None:
        collision_mingap = ET.SubElement(processing_element, "collision.mingap-factor")
    collision_mingap.set("value", "1")

    # 设置 collision.check-junctions
    collision_check_junctions = processing_element.find("collision.check-junctions")
    if collision_check_junctions is None:
        collision_check_junctions = ET.SubElement(processing_element, "collision.check-junctions")
    collision_check_junctions.set("value", "True")

    # 设置 collision.action
    collision_action = processing_element.find("collision.action")
    if collision_action is None:
        collision_action = ET.SubElement(processing_element, "collision.action")
    collision_action.set("value", "warn")

    # 添加 report 部分（如果不存在）
    report_element = root.find("report")
    if report_element is None:
        report_element = ET.SubElement(root, "report")
        verbose = ET.SubElement(report_element, "verbose")
        verbose.set("value", "false")
        no_step_log = ET.SubElement(report_element, "no-step-log")
        no_step_log.set("value", "true")
        no_warnings = ET.SubElement(report_element, "no-warnings")
        no_warnings.set("value", "true")

    # 格式化并写入 XML（使用 minidom 进行格式化）
    try:
        import xml.dom.minidom
        xml_string = ET.tostring(root, encoding="utf-8")
        dom = xml.dom.minidom.parseString(xml_string)
        pretty_xml = dom.toprettyxml(indent="\t", encoding="utf-8")
        # 移除 minidom 添加的额外空行
        lines = [line for line in pretty_xml.decode("utf-8").split("\n") if line.strip()]
        formatted_xml = "\n".join(lines)
        with open(sumo_cfg_path, "w", encoding="utf-8") as f:
            f.write(formatted_xml)
    except Exception:
        # 如果格式化失败，使用原始方式写入
        tree.write(sumo_cfg_path, encoding="utf-8", xml_declaration=True)
def _rename_converter_outputs(output_folder: Path, original_name: str, scenario_name: str):
    """将转换器生成的文件统一重命名前缀"""
    if not original_name or original_name == scenario_name:
        return

    for file_path in output_folder.iterdir():
        if not file_path.is_file():
            continue
        if not file_path.name.startswith(original_name):
            continue

        suffix = file_path.name[len(original_name) :]
        target_path = file_path.with_name(f"{scenario_name}{suffix}")
        if target_path.exists():
            target_path.unlink()
        shutil.move(str(file_path), str(target_path))


def _remove_all_traffic_lights(lanelet_network):
    """移除场景中的所有交通灯，以避免转换时的错误"""
    try:
        lanelets = getattr(lanelet_network, "lanelets", [])
        for lanelet in lanelets:
            if getattr(lanelet, "traffic_lights", None):
                lanelet.traffic_lights = set()

        if hasattr(lanelet_network, "_traffic_lights"):
            lanelet_network._traffic_lights = {}

        cleanup_method = getattr(lanelet_network, "cleanup_traffic_lights", None)
        if callable(cleanup_method):
            cleanup_method()
    except Exception as exc:
        print(f"  警告: 移除交通灯时出现问题: {exc}")


def _generate_routes_from_solution(
    solution_file: str,
    scenario,
    converter: CR2SumoMapConverter,
    planning_problem_set,
    output_folder: Path,
    scenario_name: str,
    car_follow_model: str = "Krauss",
    num_pedestrians: int = 0,
    pedestrian_speed: float = 1.3,
) -> Optional[str]:
    """根据 CommonRoad solution 生成 SUMO 车辆和行人路由文件
    
    根据配置参数将 solution 中的对象分配为：
    - 第一个为 ego vehicle
    - 接下来 num_pedestrians 个为 pedestrian
    - 剩下的为普通 vehicle
    """
    if solution_file is None:
        return None
        
    solution = CommonRoadSolutionReader.open(solution_file)

    lanelet_network = scenario.lanelet_network
    route_entries = []

    # 确保 converter 已经完成转换，new_edges 已生成
    if not hasattr(converter, "new_edges") or not converter.new_edges:
        print("  警告: 转换器尚未完成转换，无法生成 solution 路由文件")
        return None
    
    # 根据配置分配对象类型
    for idx, planning_problem_solution in enumerate(solution.planning_problem_solutions):
        trajectory = planning_problem_solution.trajectory
        edges = _edge_sequence_from_states(
            trajectory.state_list,
            lanelet_network,
            converter,
        )
        if not edges:
            print(f"  警告: Planning problem {idx} 的轨迹无法映射到有效的 SUMO edge，跳过")
            continue
            
        depart_time = getattr(trajectory.state_list[0], "time_step", 0) if trajectory.state_list else 0
        planning_problem_id = planning_problem_solution.planning_problem_id
        
        # 根据配置和索引确定对象类型
        # 第一个为 ego vehicle，接下来 num_pedestrians 个为 pedestrian，剩下的为普通 vehicle
        if idx == 0:
            object_type = "ego"
        elif 1 <= idx <= num_pedestrians:
            object_type = "pedestrian"
        else:
            object_type = "vehicle"

        route_entries.append(
            {
                "planning_problem_id": planning_problem_id if planning_problem_id is not None else idx,
                "depart_time": depart_time,
                "edges": edges,
                "object_type": object_type,
            }
        )

    if not route_entries:
        print("  警告: Solution 中未生成有效的路线，跳过 solution 路由文件")
        return None

    route_file_path = output_folder / f"{scenario_name}.solution.rou.xml"
    with open(route_file_path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n')
        
        # 定义车辆类型
        f.write(f'    <vType id="solution_ego" accel="2.5" decel="7.5" length="4.5" maxSpeed="35" guiShape="passenger" carFollowModel="{car_follow_model}"/>\n')
        f.write(f'    <vType id="solution_vehicle" accel="2.5" decel="7.5" length="4.5" maxSpeed="35" guiShape="passenger" carFollowModel="{car_follow_model}"/>\n')
        
        # 检查是否有行人需要定义类型
        has_pedestrians = any(entry["object_type"] == "pedestrian" for entry in route_entries)
        if has_pedestrians:
            f.write(f'    <vType id="pedestrian" vClass="pedestrian" speed="{pedestrian_speed:.2f}" length="0.5" width="0.5" guiShape="pedestrian"/>\n')

        # 写入车辆和行人
        for entry in route_entries:
            object_type = entry["object_type"]
            planning_problem_id = entry["planning_problem_id"]
            edges_str = " ".join(entry["edges"])
            depart = entry["depart_time"]
            
            if object_type == "pedestrian":
                person_id = f"ped_{planning_problem_id}"
                f.write(f'    <person id="{person_id}" type="pedestrian" depart="{depart:.2f}">\n')
                f.write(f'        <walk edges="{edges_str}" speed="{pedestrian_speed:.2f}"/>\n')
                f.write("    </person>\n")
            else:
                vehicle_prefix = "egoVehicle" if object_type == "ego" else "vehicle"
                vehicle_type = "solution_ego" if object_type == "ego" else "solution_vehicle"
                vehicle_id = f"{vehicle_prefix}_{planning_problem_id}"
                f.write(f'    <vehicle id="{vehicle_id}" type="{vehicle_type}" depart="{depart}" departLane="free" departPos="random">\n')
                f.write(f'        <route edges="{edges_str}"/>\n')
                f.write("    </vehicle>\n")

        f.write("</routes>\n")

    num_vehicles_written = sum(1 for e in route_entries if e["object_type"] != "pedestrian")
    num_pedestrians_written = sum(1 for e in route_entries if e["object_type"] == "pedestrian")
    print(f"  ✓ 已根据 solution 生成路由文件: {route_file_path.name} (车辆: {num_vehicles_written}, 行人: {num_pedestrians_written})")
    return route_file_path.name


def _ensure_pedestrian_access_on_net(net_file_path: Path) -> None:
    if not net_file_path.exists():
        print(f"  警告: 找不到网络文件 {net_file_path}，无法设置行人权限。")
        return

    try:
        tree = ET.parse(net_file_path)
    except Exception as exc:
        print(f"  警告: 解析网络文件失败 {net_file_path}: {exc}")
        return

    root = tree.getroot()
    changed = False
    for lane in root.findall(".//lane"):
        disallow = lane.attrib.get("disallow", "")
        disallow_tokens = disallow.split() if disallow else []

        if "pedestrian" in disallow_tokens:
            disallow_tokens = [token for token in disallow_tokens if token != "pedestrian"]
            if disallow_tokens:
                lane.attrib["disallow"] = " ".join(disallow_tokens)
            else:
                lane.attrib.pop("disallow", None)
            changed = True

    if not changed:
        return

    try:
        import xml.dom.minidom

        xml_string = ET.tostring(root, encoding="utf-8")
        dom = xml.dom.minidom.parseString(xml_string)
        pretty_xml = dom.toprettyxml(indent="\t", encoding="utf-8")
        lines = [line for line in pretty_xml.decode("utf-8").split("\n") if line.strip()]
        formatted_xml = "\n".join(lines)
        with open(net_file_path, "w", encoding="utf-8") as f:
            f.write(formatted_xml)
    except Exception:
        tree.write(net_file_path, encoding="utf-8", xml_declaration=True)

    print(f"  ✓ 已更新 {net_file_path.name}，允许行人通行。")


def _extract_pedestrian_edge_graph(
    net_file_path: Path,
) -> Optional[Tuple[Dict[str, Dict[str, str]], Dict[str, List[str]]]]:
    if not net_file_path.exists():
        print(f"  警告: 找不到网络文件 {net_file_path}，无法生成行人。")
        return None

    try:
        tree = ET.parse(net_file_path)
    except Exception as exc:
        print(f"  警告: 解析网络文件失败 {net_file_path}: {exc}")
        return None

    root = tree.getroot()
    edge_info: Dict[str, Dict[str, str]] = {}
    edges_from_node: Dict[str, List[str]] = defaultdict(list)

    for edge in root.findall("edge"):
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

    if not edge_info:
        print("  警告: 网络中未找到可用的边，跳过行人生成。")
        return None

    return edge_info, edges_from_node




def _edge_sequence_from_states(state_list, lanelet_network, converter: CR2SumoMapConverter):
    """
    从状态序列生成 edge 序列，只使用最终 SUMO 网络中存在的 edge
    
    注意：转换过程中某些 edge 可能被合并或删除，所以必须检查 edge 是否在 new_edges 中
    """
    edges = []
    # 获取最终网络中的 edge（转换后保留的 edge）
    new_edges = getattr(converter, "new_edges", {})
    
    for state in state_list:
        position = getattr(state, "position", None)
        if position is None:
            continue
        try:
            lanelet_ids = lanelet_network.find_lanelet_by_position([position])[0]
        except Exception:
            continue

        edge_name = None
        for lanelet_id in lanelet_ids:
            edge_id = converter.lanelet_id2edge_id.get(lanelet_id)
            if edge_id is None:
                continue
                
            # 检查 edge 是否存在于最终网络中
            if edge_id not in new_edges:
                # 这个 edge 在转换过程中被删除了，跳过
                continue
                
            edge_obj = new_edges[edge_id]
            if edge_obj is not None and hasattr(edge_obj, "id"):
                edge_name = str(edge_obj.id)
            else:
                edge_name = str(edge_id)
            break

        if edge_name is None:
            continue

        # 避免重复添加相同的 edge
        if not edges or edges[-1] != edge_name:
            edges.append(edge_name)

    edges = _repair_edge_sequence_for_connectivity(edges, converter)
    return edges


def _repair_edge_sequence_for_connectivity(
        edge_list: List[str], converter: CR2SumoMapConverter
) -> List[str]:
    if len(edge_list) <= 1:
        return edge_list

    edge_lookup: Dict[str, Edge] = {}
    for raw_id, edge in converter.new_edges.items():
        identifier = str(getattr(edge, "id", raw_id))
        edge_lookup[identifier] = edge

    node_to_edges: Dict[int, List[str]] = defaultdict(list)
    for key, edge in edge_lookup.items():
        if edge.from_node is not None:
            node_to_edges[edge.from_node.id].append(key)

    successors: Dict[str, List[str]] = {}
    for key, edge in edge_lookup.items():
        node_id = edge.to_node.id if edge.to_node is not None else None
        successors[key] = node_to_edges.get(node_id, []) if node_id is not None else []

    def edges_connected(a: str, b: str) -> bool:
        ea = edge_lookup.get(a)
        eb = edge_lookup.get(b)
        if ea is None or eb is None:
            return False
        if ea.to_node is None or eb.from_node is None:
            return False
        return ea.to_node.id == eb.from_node.id

    def shortest_path(start: str, goal: str) -> Optional[List[str]]:
        if start not in successors or goal not in edge_lookup:
            return None
        queue = deque([(start, [start])])
        visited = {start}
        while queue:
            current, path = queue.popleft()
            for nxt in successors.get(current, []):
                if nxt == goal:
                    return path + [nxt]
                if nxt not in visited:
                    visited.add(nxt)
                    queue.append((nxt, path + [nxt]))
        return None

    repaired: List[str] = [edge_list[0]]
    for next_edge in edge_list[1:]:
        last_edge = repaired[-1]
        if edges_connected(last_edge, next_edge):
            repaired.append(next_edge)
            continue
        bridging_path = shortest_path(last_edge, next_edge)
        if not bridging_path:
            return []
        repaired.extend(bridging_path[1:])

    return repaired


if __name__ == "__main__":
    import argparse
    from pathlib import Path

    def _default_paths_from_name(scenario_name: str):
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
        solution_file = scenario_folder / f"{scenario_name}_solution.xml"
        if not solution_file.exists():
            candidates = sorted(scenario_folder.glob("*_solution.xml"))
            if len(candidates) == 1:
                solution_file = candidates[0]
            elif len(candidates) > 1:
                names = ", ".join(str(path.name) for path in candidates)
                raise ValueError(
                    f"{scenario_folder} 中存在多个 *_solution.xml 文件 ({names})，请使用 --solution_file 指定目标文件。"
                )
        return commonroad_file, scenario_folder, solution_file

    parser = argparse.ArgumentParser(
        description="从CommonRoad场景生成SUMO仿真配置文件"
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
        "--simulation_steps",
        type=int,
        default=500,
        help="仿真步数（默认500）"
    )
    parser.add_argument(
        "--step_length",
        type=float,
        default=0.1,
        help="每步时间长度（秒，默认0.1）"
    )
    parser.add_argument(
        "--solution_file",
        type=str,
        default=None,
        help="解决方案文件路径 (.solution.xml)"
    )
    parser.add_argument(
        "--car_follow_model",
        type=str,
        default="EIDM",
        choices=["Krauss", "KraussOrig1", "KraussPS", "KraussAccident", "IDM", "IDMM", "IDMPlus", "IDMVanilla", 
                 "W99", "PWagner2009", "PWagner", "BKerner", "SKraussCL", "Gipps", "Helly", "Linear", 
                 "Newell", "FullVelocityDifference", "SmartSK", "ACC"],
        help="跟驰模型名称（默认: Krauss）。可选: Krauss, IDM, IDMM, W99, PWagner2009, BKerner, Gipps, Linear, Newell 等"
    )
    parser.add_argument(
        "--num_pedestrians",
        type=int,
        default=2,
        help="随机行人数量（默认 0，不生成）"
    )
    parser.add_argument(
        "--pedestrian_speed",
        type=float,
        default=1.3,
        help="随机行人速度（m/s，默认 1.3）"
    )
    parser.add_argument(
        "--pedestrian_depart_interval",
        type=float,
        default=2.0,
        help="随机行人出发间隔（秒，默认 2.0）"
    )
    parser.add_argument(
        "--pedestrian_walk_min",
        type=int,
        default=3,
        help="随机行人路径的最少边数（默认 3）"
    )
    parser.add_argument(
        "--pedestrian_walk_max",
        type=int,
        default=6,
        help="随机行人路径的最多边数（默认 6）"
    )
    parser.add_argument(
        "--pedestrian_seed",
        type=int,
        default=None,
        help="随机行人生成用的随机种子（默认随机）"
    )
    args = parser.parse_args()

    commonroad_file = args.commonroad_file
    output_folder = args.output_folder
    solution_file = args.solution_file

    if args.scenario_name and (not commonroad_file or not output_folder or not solution_file):
        default_cr, default_folder, default_solution = _default_paths_from_name(args.scenario_name)
        commonroad_file = commonroad_file or str(default_cr)
        output_folder = output_folder or str(default_folder)
        solution_file = solution_file or (str(default_solution) if default_solution.exists() else None)

    if not commonroad_file:
        raise ValueError("必须提供 --commonroad_file，或通过 --scenario_name 推断。")
    if not output_folder:
        raise ValueError("必须提供 --output_folder，或通过 --scenario_name 推断。")

    generate_sumo_files_from_commonroad(
        commonroad_file,
        output_folder,
        args.scenario_name,
        args.simulation_steps,
        args.step_length,
        solution_file=solution_file,
        car_follow_model=args.car_follow_model,
        num_random_pedestrians=args.num_pedestrians,
        pedestrian_speed=args.pedestrian_speed,
        pedestrian_depart_interval=args.pedestrian_depart_interval,
        pedestrian_walk_min=args.pedestrian_walk_min,
        pedestrian_walk_max=args.pedestrian_walk_max,
        pedestrian_seed=args.pedestrian_seed,
    )

