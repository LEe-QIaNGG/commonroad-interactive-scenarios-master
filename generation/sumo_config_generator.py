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
from typing import Optional, Dict, List
from pathlib import Path

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
    :param default_config_kwargs: DefaultConfig的其他参数
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

    route_file_name = None
    if solution_file:
        generated_route = _generate_routes_from_solution(
            solution_file,
            scenario,
            converter,
            planning_problem_set,
            output_folder,
            scenario_name,
        )
        if generated_route:
            route_file_name = generated_route

    if not route_file_name:
        route_file_name = _find_existing_route_file(output_folder, scenario_name)

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
    print(f"  - {route_file_name}")
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
    """更新SUMO配置文件以确保引用正确的网络和路由文件"""
    try:
        tree = ET.parse(sumo_cfg_path)
        root = tree.getroot()
    except Exception:
        create_sumo_config_file(scenario_name, route_file_name, sumo_cfg_path, step_length)
        return

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


def _find_existing_route_file(output_folder: Path, scenario_name: str) -> str:
    """查找默认生成的路由文件"""
    candidates = sorted(output_folder.glob(f"{scenario_name}*.rou.xml"))
    if candidates:
        return candidates[0].name
    return f"{scenario_name}.vehicle.rou.xml"


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
) -> Optional[str]:
    """根据 CommonRoad solution 生成 SUMO 车辆路由文件"""
    try:
        solution = CommonRoadSolutionReader.open(solution_file)
    except Exception as exc:
        print(f"  警告: 无法读取 solution 文件 {solution_file}: {exc}")
        return None

    lanelet_network = scenario.lanelet_network
    route_entries = []

    # 确保 converter 已经完成转换，new_edges 已生成
    if not hasattr(converter, "new_edges") or not converter.new_edges:
        print("  警告: 转换器尚未完成转换，无法生成 solution 路由文件")
        return None
    
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
        
        # 验证 route 至少包含 2 个 edge（起点和终点）
        if len(edges) < 2:
            print(f"  警告: Planning problem {idx} 的 route 太短（只有 {len(edges)} 个 edge），跳过")
            continue
            
        depart_time = getattr(trajectory.state_list[0], "time_step", 0) if trajectory.state_list else 0
        planning_problem_id = planning_problem_solution.planning_problem_id
        is_ego = False
        if planning_problem_set is not None and planning_problem_id is not None:
            is_ego = planning_problem_id in planning_problem_set.planning_problem_dict

        route_entries.append(
            {
                "planning_problem_id": planning_problem_id if planning_problem_id is not None else idx,
                "depart_time": depart_time,
                "edges": edges,
                "is_ego": is_ego,
            }
        )

    if not route_entries:
        print("  警告: Solution 中未生成有效的路线，跳过 solution 路由文件")
        return None

    route_file_path = output_folder / f"{scenario_name}.solution.rou.xml"
    with open(route_file_path, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n')
        f.write('    <vType id="solution_ego" accel="2.5" decel="7.5" length="4.5" maxSpeed="35" guiShape="passenger"/>\n')
        f.write('    <vType id="solution_vehicle" accel="2.5" decel="7.5" length="4.5" maxSpeed="35" guiShape="passenger"/>\n')

        for entry in route_entries:
            is_ego = entry.get("is_ego", False)
            vehicle_prefix = "egoVehicle" if is_ego else "vehicle"
            vehicle_type = "solution_ego" if is_ego else "solution_vehicle"
            vehicle_id = f"{vehicle_prefix}_{entry['planning_problem_id']}"
            edges_str = " ".join(entry["edges"])
            depart = entry["depart_time"]
            f.write(f'    <vehicle id="{vehicle_id}" type="{vehicle_type}" depart="{depart}" departLane="free" departPos="random">\n')
            f.write(f'        <route edges="{edges_str}"/>\n')
            f.write("    </vehicle>\n")

        f.write("</routes>\n")

    print(f"  ✓ 已根据 solution 生成路由文件: {route_file_path.name}")
    return route_file_path.name


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


def create_sumo_config_file(
    scenario_name: str,
    route_file_name: str,
    output_path: str,
    step_length: float = 0.1,
) -> None:
    """
    创建SUMO配置文件 (.sumo.cfg)

    :param scenario_name: 场景名称
    :param output_path: 输出文件路径
    :param step_length: 每步时间长度（秒）
    """
    config_content = f"""<?xml version="1.0" ?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">
	<input>
		<net-file value="{scenario_name}.net.xml"/>
		<route-files value="{route_file_name}"/>
	</input>
	<time>
		<begin value="0"/>
		<step-length value="{step_length}"/>
	</time>
	<report>
		<verbose value="false"/>
		<no-step-log value="true"/>
		<no-warnings value="true"/>
	</report>
	<processing>
		<lateral-resolution value="1.0"/>
		<max-depart-delay value="5"/>
		<time-to-teleport value="-1"/>
		<collision.mingap-factor value="1"/>
		<collision.check-junctions value="false"/>
		<collision.action value="none"/>
	</processing>
</configuration>"""

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(config_content)


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
        solution_file = scenario_folder / f"{scenario_name}_solution.xml"
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
    )

