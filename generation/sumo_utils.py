"""
SUMO 文件生成工具函数

包含 sumo_from_graph.py 和 sumo_from_solution.py 中共同使用的工具函数。

作者: Auto-generated
"""

__author__ = "Auto-generated"
__version__ = "0.1"

import shutil
import xml.etree.ElementTree as ET
import xml.dom.minidom
from pathlib import Path


def remove_all_traffic_lights(lanelet_network):
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


def rename_converter_outputs(output_folder: Path, original_name: str, scenario_name: str):
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


def update_sumo_config_file(
    sumo_cfg_path: Path,
    scenario_name: str,
    route_file_name: str,
    step_length: float,
):
    """更新或创建 SUMO 配置文件
    
    包含所有避免插入阶段碰撞的配置参数。
    
    :param sumo_cfg_path: SUMO配置文件路径（Path对象或字符串）
    :param scenario_name: 场景名称
    :param route_file_name: 路由文件名
    :param step_length: 仿真步长（秒）
    """
    # 转换为 Path 对象
    if isinstance(sumo_cfg_path, str):
        sumo_cfg_path = Path(sumo_cfg_path)
    
    if sumo_cfg_path.exists():
        tree = ET.parse(sumo_cfg_path)
        root = tree.getroot()
    else:
        root = ET.Element("configuration")
        tree = ET.ElementTree(root)

    # 更新 input 部分
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

    # 更新 time 部分
    time_element = root.find("time")
    if time_element is None:
        time_element = ET.SubElement(root, "time")

    step_length_element = time_element.find("step-length")
    if step_length_element is None:
        step_length_element = ET.SubElement(time_element, "step-length")
    step_length_element.set("value", str(step_length))

    # 添加或更新 processing 部分
    processing_element = root.find("processing")
    if processing_element is None:
        processing_element = ET.SubElement(root, "processing")

    # 设置常用配置（包括避免插入阶段碰撞的配置）
    for config_name, config_value in [
        ("lateral-resolution", "1.0"),
        ("max-depart-delay", "5"),
        ("time-to-teleport", "-1"),
        ("time-to-teleport.remove", "true"),
        ("collision.mingap-factor", "1"),
        ("collision.check-junctions", "True"),
        ("collision.action", "warn"),
        ("eager-insert", "true"),
        ("sloppy-insert", "true"),
        ("random-depart-offset", "2.0"),
    ]:
        config_element = processing_element.find(config_name)
        if config_element is None:
            config_element = ET.SubElement(processing_element, config_name)
        config_element.set("value", config_value)

    # 添加 report 部分
    report_element = root.find("report")
    if report_element is None:
        report_element = ET.SubElement(root, "report")
        for config_name, config_value in [
            ("verbose", "True"),
            ("no-step-log", "true"),
            ("no-warnings", "False"),
        ]:
            config_elem = ET.SubElement(report_element, config_name)
            config_elem.set("value", config_value)

    # 格式化并写入 XML
    try:
        xml_string = ET.tostring(root, encoding="utf-8")
        dom = xml.dom.minidom.parseString(xml_string)
        pretty_xml = dom.toprettyxml(indent="\t", encoding="utf-8")
        lines = [line for line in pretty_xml.decode("utf-8").split("\n") if line.strip()]
        formatted_xml = "\n".join(lines)
        with open(sumo_cfg_path, "w", encoding="utf-8") as f:
            f.write(formatted_xml)
    except Exception:
        tree.write(sumo_cfg_path, encoding="utf-8", xml_declaration=True)

