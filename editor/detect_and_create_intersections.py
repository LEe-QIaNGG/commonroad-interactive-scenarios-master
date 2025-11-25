"""
自动检测并创建 intersection

该脚本会分析 lanelet 网络，找出形成 intersection 的 lanelet，
然后自动创建 Intersection 对象。
"""

import numpy as np
from pathlib import Path
from typing import Dict, List, Set, Tuple
from collections import defaultdict

from commonroad.scenario.intersection import Intersection, IntersectionIncomingElement
from commonroad.scenario.lanelet import Lanelet

from crdesigner.common.file_reader import CRDesignerFileReader
from crdesigner.common.file_writer import CRDesignerFileWriter, OverwriteExistingFile


def detect_intersection_candidates(scenario) -> List[Dict[int, List[int]]]:
    """
    检测可能形成 intersection 的 lanelet 组
    
    策略：
    1. 找到有多个 successor 的 lanelet（分叉点）
    2. 找到多个 lanelet 汇聚的地方（合并点）
    3. 分析哪些 lanelet 的 successor 相互交叉
    
    :param scenario: CommonRoad Scenario 对象
    :return: intersection_map 列表，每个元素是一个字典 {incoming_lanelet_id: [successor_ids]}
    """
    lanelet_network = scenario.lanelet_network
    intersection_maps = []
    
    # 策略1: 找到有多个 successor 的 lanelet
    # 这些通常是进入 intersection 的 lanelet
    multi_successor_lanelets = {}
    for lanelet in lanelet_network.lanelets:
        if len(lanelet.successor) > 1:
            multi_successor_lanelets[lanelet.lanelet_id] = lanelet.successor
    
    print(f"找到 {len(multi_successor_lanelets)} 个有多个 successor 的 lanelet")
    
    # 策略2: 找到多个 lanelet 汇聚的地方
    # 统计每个 lanelet 有多少个 predecessor
    predecessor_count = defaultdict(int)
    for lanelet in lanelet_network.lanelets:
        for pred_id in lanelet.predecessor:
            predecessor_count[pred_id] += 1
    
    # 找到有多个 predecessor 的 lanelet（合并点）
    merge_points = {lid: count for lid, count in predecessor_count.items() if count > 1}
    print(f"找到 {len(merge_points)} 个合并点")
    
    # 策略3: 分析 lanelet 的几何交叉
    # 找到那些 successor 相互交叉的 incoming lanelet
    intersection_groups = []
    processed_lanelets = set()
    
    for incoming_id, successors in multi_successor_lanelets.items():
        if incoming_id in processed_lanelets:
            continue
        
        # 检查这些 successor 是否与其他 lanelet 的 successor 交叉
        incoming_lanelet = lanelet_network.find_lanelet_by_id(incoming_id)
        if incoming_lanelet is None:
            continue
        
        # 收集所有相关的 incoming lanelet（通过 successor 交叉关系）
        related_incomings = {incoming_id}
        related_successors = set(successors)
        
        # 查找其他 incoming lanelet，它们的 successor 与当前 successor 交叉
        for other_incoming_id, other_successors in multi_successor_lanelets.items():
            if other_incoming_id == incoming_id or other_incoming_id in processed_lanelets:
                continue
            
            other_incoming_lanelet = lanelet_network.find_lanelet_by_id(other_incoming_id)
            if other_incoming_lanelet is None:
                continue
            
            # 检查几何交叉
            has_intersection = False
            for succ_id in successors:
                succ_lanelet = lanelet_network.find_lanelet_by_id(succ_id)
                if succ_lanelet is None:
                    continue
                
                for other_succ_id in other_successors:
                    other_succ_lanelet = lanelet_network.find_lanelet_by_id(other_succ_id)
                    if other_succ_lanelet is None:
                        continue
                    
                    # 检查多边形是否交叉
                    try:
                        if succ_lanelet.polygon.shapely_object.intersects(
                            other_succ_lanelet.polygon.shapely_object
                        ):
                            has_intersection = True
                            break
                    except:
                        pass
                
                if has_intersection:
                    break
            
            if has_intersection:
                related_incomings.add(other_incoming_id)
                related_successors.update(other_successors)
                processed_lanelets.add(other_incoming_id)
        
        # 如果找到至少2个相关的 incoming，形成一个 intersection
        if len(related_incomings) >= 2:
            intersection_map = {}
            for inc_id in related_incomings:
                inc_lanelet = lanelet_network.find_lanelet_by_id(inc_id)
                if inc_lanelet:
                    intersection_map[inc_id] = list(inc_lanelet.successor)
            
            if len(intersection_map) >= 2:
                intersection_groups.append(intersection_map)
                processed_lanelets.update(related_incomings)
    
    print(f"检测到 {len(intersection_groups)} 个潜在的 intersection")
    return intersection_groups


def classify_successor_direction(
    incoming_lanelet: Lanelet, 
    successor_lanelet: Lanelet
) -> str:
    """
    判断 successor 相对于 incoming 的方向（left, straight, right）
    
    :param incoming_lanelet: 进入 intersection 的 lanelet
    :param successor_lanelet: successor lanelet
    :return: "left", "straight", 或 "right"
    """
    # 计算 incoming 的方向向量（最后两个点的方向）
    if len(incoming_lanelet.center_vertices) < 2:
        return "straight"
    
    incoming_dir = incoming_lanelet.center_vertices[-1] - incoming_lanelet.center_vertices[-2]
    incoming_angle = np.arctan2(incoming_dir[1], incoming_dir[0])
    
    # 计算 successor 的方向向量（前两个点的方向）
    if len(successor_lanelet.center_vertices) < 2:
        return "straight"
    
    successor_dir = successor_lanelet.center_vertices[1] - successor_lanelet.center_vertices[0]
    successor_angle = np.arctan2(successor_dir[1], successor_dir[0])
    
    # 计算角度差
    angle_diff = successor_angle - incoming_angle
    # 归一化到 [-pi, pi]
    angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
    
    # 转换为度数以便判断
    angle_deg = np.degrees(angle_diff)
    
    # 判断方向
    if abs(angle_deg) < 30:  # 直行
        return "straight"
    elif 30 <= angle_deg < 150:  # 左转
        return "left"
    elif -150 <= angle_deg < -30:  # 右转
        return "right"
    else:  # 掉头（归类为左转）
        return "left"


def find_left_of_relationship(
    lanelet_network,
    incomings: List[IntersectionIncomingElement]
) -> None:
    """
    设置 incoming 之间的 left_of 关系（右让左规则）
    
    :param lanelet_network: LaneletNetwork 对象
    :param incomings: IntersectionIncomingElement 列表
    """
    if len(incomings) < 2:
        return
    
    # 计算每个 incoming 的角度
    angles = []
    for i, incoming in enumerate(incomings):
        incoming_id = list(incoming.incoming_lanelets)[0]
        incoming_lanelet = lanelet_network.find_lanelet_by_id(incoming_id)
        if incoming_lanelet is None or len(incoming_lanelet.center_vertices) < 2:
            angles.append((i, 0))
            continue
        
        # 计算 incoming 的方向向量
        if len(incoming_lanelet.center_vertices) >= 3:
            direction = incoming_lanelet.center_vertices[-1] - incoming_lanelet.center_vertices[-3]
        else:
            direction = incoming_lanelet.center_vertices[-1] - incoming_lanelet.center_vertices[-2]
        
        angle = np.arctan2(direction[1], direction[0])
        angles.append((i, angle))
    
    # 按角度排序
    angles.sort(key=lambda x: x[1])
    
    # 设置 left_of 关系（相邻且角度差小于180度的，右边的 left_of 左边的）
    for i in range(len(angles)):
        current_idx = angles[i][0]
        current_angle = angles[i][1]
        
        # 找到下一个 incoming（角度差小于180度）
        min_angle_diff = float('inf')
        left_of_idx = None
        
        for j in range(len(angles)):
            if i == j:
                continue
            other_idx = angles[j][0]
            other_angle = angles[j][1]
            
            # 计算角度差（考虑周期性）
            angle_diff = other_angle - current_angle
            if angle_diff < 0:
                angle_diff += 2 * np.pi
            
            if 0 < angle_diff < np.pi and angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                left_of_idx = other_idx
        
        if left_of_idx is not None:
            incomings[current_idx].left_of = incomings[left_of_idx].incoming_id


def create_intersection_from_map(
    scenario,
    intersection_map: Dict[int, List[int]],
    intersection_id: int
) -> Intersection:
    """
    从 intersection_map 创建 Intersection 对象
    
    :param scenario: CommonRoad Scenario 对象
    :param intersection_map: 字典 {incoming_lanelet_id: [successor_ids]}
    :param intersection_id: intersection ID
    :return: Intersection 对象
    """
    lanelet_network = scenario.lanelet_network
    incoming_elements = []
    
    # 按 incoming lanelet 分组
    for incoming_id, successor_ids in intersection_map.items():
        incoming_lanelet = lanelet_network.find_lanelet_by_id(incoming_id)
        if incoming_lanelet is None:
            continue
        
        # 分类 successor
        successor_left = set()
        successor_straight = set()
        successor_right = set()
        
        for succ_id in successor_ids:
            successor_lanelet = lanelet_network.find_lanelet_by_id(succ_id)
            if successor_lanelet is None:
                continue
            
            direction = classify_successor_direction(incoming_lanelet, successor_lanelet)
            if direction == "left":
                successor_left.add(succ_id)
            elif direction == "straight":
                successor_straight.add(succ_id)
            elif direction == "right":
                successor_right.add(succ_id)
        
        # 创建 IntersectionIncomingElement
        incoming_element = IntersectionIncomingElement(
            incoming_id=scenario.generate_object_id(),
            incoming_lanelets={incoming_id},
            successors_left=successor_left,
            successors_straight=successor_straight,
            successors_right=successor_right,
        )
        incoming_elements.append(incoming_element)
    
    # 设置 left_of 关系
    find_left_of_relationship(lanelet_network, incoming_elements)
    
    # 创建 Intersection
    intersection = Intersection(
        intersection_id=intersection_id,
        incomings=incoming_elements,
        crossings=set()  # 初始为空，后续可以添加 crosswalk
    )
    
    return intersection


def auto_detect_and_create_intersections(
    input_file: Path,
    output_file: Path,
    min_incomings: int = 2
) -> List[Intersection]:
    """
    自动检测并创建 intersection
    
    :param input_file: 输入的 CommonRoad XML 文件路径
    :param output_file: 输出的 CommonRoad XML 文件路径
    :param min_incomings: 最少需要多少个 incoming 才创建 intersection（默认2个）
    :return: 创建的 Intersection 列表
    """
    print("=" * 70)
    print("自动检测并创建 intersection")
    print("=" * 70)
    
    # 读取场景
    print(f"\n读取文件: {input_file}")
    scenario, planning_problem_set = CRDesignerFileReader(str(input_file)).open()
    
    # 检查是否已有 intersection
    existing_intersections = list(scenario.lanelet_network.intersections)
    print(f"现有 intersection 数量: {len(existing_intersections)}")
    
    if len(existing_intersections) > 0:
        print("\n⚠ 场景中已存在 intersection，将跳过已关联的 lanelet")
    
    # 检测 intersection
    print("\n检测潜在的 intersection...")
    intersection_maps = detect_intersection_candidates(scenario)
    
    if len(intersection_maps) == 0:
        print("\n⚠ 未检测到 intersection")
        return []
    
    # 创建 intersection
    created_intersections = []
    for i, intersection_map in enumerate(intersection_maps):
        if len(intersection_map) < min_incomings:
            print(f"\n跳过 intersection {i+1}: incoming 数量不足 ({len(intersection_map)} < {min_incomings})")
            continue
        
        intersection_id = scenario.generate_object_id()
        print(f"\n创建 intersection {i+1} (ID: {intersection_id}):")
        print(f"  - Incoming lanelets: {len(intersection_map)} 个")
        for inc_id, succ_ids in intersection_map.items():
            print(f"    Lanelet {inc_id} -> {len(succ_ids)} 个 successors")
        
        try:
            intersection = create_intersection_from_map(
                scenario, intersection_map, intersection_id
            )
            scenario.lanelet_network.add_intersection(intersection)
            created_intersections.append(intersection)
            print(f"  ✓ 成功创建 intersection {intersection_id}")
        except Exception as e:
            print(f"  ✗ 创建失败: {e}")
            import traceback
            traceback.print_exc()
    
    # 保存文件
    if len(created_intersections) > 0:
        print(f"\n保存文件: {output_file}")
        writer = CRDesignerFileWriter(scenario, planning_problem_set)
        writer.write_to_file(str(output_file), OverwriteExistingFile.ALWAYS)
        print(f"✓ 已创建 {len(created_intersections)} 个 intersection")
    else:
        print("\n⚠ 未创建任何 intersection")
    
    return created_intersections


# ==================== 主程序 ====================

if __name__ == "__main__":
    # 设置文件路径
    input_file = Path("interactive_scenarios/CHN_TST-2_1_T-1/CHN_TST-2_1_T-1.cr.xml")
    output_file = Path("interactive_scenarios/CHN_TST-2_1_T-1/CHN_TST-2_1_T-1_with_intersections.cr.xml")
    
    try:
        intersections = auto_detect_and_create_intersections(
            input_file=input_file,
            output_file=output_file,
            min_incomings=2  # 至少需要2个 incoming
        )
        
        print("\n" + "=" * 70)
        print("完成！")
        print("=" * 70)
        print(f"\n创建的 intersection 数量: {len(intersections)}")
        print(f"输出文件: {output_file}")
        
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()

