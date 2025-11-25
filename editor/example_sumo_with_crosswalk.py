"""
示例：在 CommonRoad 场景的每个 intersection 上自动添加 crosswalk lanelet。

生成后的 crosswalk 可以在后续转换为 SUMO 时自动变成 crossing / walkingArea。
"""

import numpy as np
from pathlib import Path
from typing import Optional, Tuple

from commonroad.scenario.lanelet import LaneletType, RoadUser

from crdesigner.common.file_reader import CRDesignerFileReader
from crdesigner.common.file_writer import CRDesignerFileWriter, OverwriteExistingFile
from crdesigner.ui.gui.utilities.map_creator import MapCreator


def _compute_crosswalk_pose(intersection, lanelet_network) -> Optional[Tuple[np.ndarray, float]]:
    """返回 (center, angle)。若 intersection 无合适几何则返回 None。"""
    all_points = []
    for incoming in intersection.incomings:
        for lanelet_id in incoming.incoming_lanelets:
            lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
            if lanelet:
                all_points.append(lanelet.center_vertices[-1])

    if not all_points:
        return None

    center = np.mean(np.array(all_points), axis=0)

    crosswalk_angle = np.pi / 2
    for incoming in intersection.incomings:
        if incoming.incoming_lanelets:
            lanelet = lanelet_network.find_lanelet_by_id(
                next(iter(incoming.incoming_lanelets))
            )
            if lanelet and len(lanelet.center_vertices) >= 2:
                direction = lanelet.center_vertices[-1] - lanelet.center_vertices[-2]
                crosswalk_angle = np.arctan2(direction[1], direction[0]) + np.pi / 2
                break

    return center, crosswalk_angle


def add_crosswalks_to_all_intersections(
    input_cr_file: Path,
    output_cr_file: Path,
    crosswalk_width: float = 4.0,
    crosswalk_length: float = 8.0,
) -> Path:
    """为场景中所有 intersection 添加 crosswalk 并保存。"""
    scenario, planning_problem_set = CRDesignerFileReader(str(input_cr_file)).open()

    # intersection_ids = [
    #     inter.intersection_id for inter in scenario.lanelet_network.intersections
    # ]
    intersection_ids = [scenario.lanelet_network.intersections[0].intersection_id]
    if not intersection_ids:
        raise ValueError("场景中没有 intersection，无法添加 crosswalk。")

    print(
        f"在 {len(intersection_ids)} 个 intersection 上添加 crosswalk "
        f"(width={crosswalk_width}, length={crosswalk_length})"
    )

    from commonroad.scenario.intersection import Intersection, IntersectionIncomingElement

    for inter_id in intersection_ids:
        intersection = scenario.lanelet_network.find_intersection_by_id(inter_id)
        if intersection is None:
            print(f"  - Intersection {inter_id} 不存在，跳过。")
            continue

        pose = _compute_crosswalk_pose(intersection, scenario.lanelet_network)
        if pose is None:
            print(f"  - Intersection {inter_id} 缺少几何信息，跳过。")
            continue

        center, angle = pose
        crosswalk_id = scenario.generate_object_id()
        crosswalk = MapCreator.create_straight(
            width=crosswalk_width,
            length=crosswalk_length,
            num_vertices=max(5, int(crosswalk_length / 0.5)),
            lanelet_id=crosswalk_id,
            lanelet_types={LaneletType.CROSSWALK},
            road_user_bidirectional={RoadUser.PEDESTRIAN},
        )
        crosswalk.translate_rotate(center, angle)
        first_point = crosswalk.center_vertices[0]
        last_point = crosswalk.center_vertices[-1]
        print(
            f"    -> crosswalk lanelet {crosswalk_id}: "
            f"start=({first_point[0]:.2f},{first_point[1]:.2f}), "
            f"end=({last_point[0]:.2f},{last_point[1]:.2f})"
        )
        scenario.add_objects(crosswalk)

        new_incomings = [
            IntersectionIncomingElement(
                inc.incoming_id,
                inc.incoming_lanelets,
                inc.successors_left,
                inc.successors_straight,
                inc.successors_right,
            )
            for inc in intersection.incomings
        ]
        existing_crossings = set(getattr(intersection, "crossings", []))
        existing_crossings.add(crosswalk_id)

        scenario.lanelet_network.remove_intersection(inter_id)
        scenario.lanelet_network.add_intersection(
            Intersection(inter_id, new_incomings, list(existing_crossings))
        )
        print(f"  - 已在 intersection {inter_id} 添加 crosswalk (id={crosswalk_id}).")

    writer = CRDesignerFileWriter(scenario, planning_problem_set)
    writer.write_to_file(str(output_cr_file), OverwriteExistingFile.ALWAYS)
    print(f"\n✓ 完成，输出文件: {output_cr_file}")
    return output_cr_file


# ==================== 主程序示例 ====================

if __name__ == "__main__":
    base_path = Path("interactive_scenarios/CHN_TST-2_1_T-1")
    input_cr_file = base_path / "CHN_TST-2_1_T-1_with_intersections.cr.xml"
    output_cr_file = base_path / "CHN_TST-2_1_T-1_with_crosswalk.cr.xml"

    try:
        add_crosswalks_to_all_intersections(
            input_cr_file=input_cr_file,
            output_cr_file=output_cr_file,
            crosswalk_width=4.0,
            crosswalk_length=8.0,
        )
    except Exception as exc:
        print(f"\n✗ 错误: {exc}")
        import traceback

        traceback.print_exc()

