"""
检查 CommonRoad 场景文件中是否有 intersection
"""

from pathlib import Path
from crdesigner.common.file_reader import CRDesignerFileReader

# 文件路径
input_file = Path("tutorials/conversion_examples/CHN_TST-2_1_T-1.xml")

print("=" * 70)
print("检查 CommonRoad 场景中的 intersection")
print("=" * 70)
print(f"\n文件: {input_file}")

try:
    # 读取场景
    scenario, planning_problem_set = CRDesignerFileReader(str(input_file)).open()
    
    # 获取所有 intersection
    intersections = list(scenario.lanelet_network.intersections)
    
    print(f"\n✓ 文件读取成功")
    print(f"\nIntersection 数量: {len(intersections)}")
    
    if len(intersections) == 0:
        print("\n⚠ 该场景中没有 intersection")
    else:
        print("\n" + "-" * 70)
        print("Intersection 详细信息:")
        print("-" * 70)
        
        for i, intersection in enumerate(intersections, 1):
            print(f"\n[{i}] Intersection ID: {intersection.intersection_id}")
            print(f"    Incoming 数量: {len(intersection.incomings)}")
            
            # 计算 intersection 中心位置（近似）
            all_points = []
            for incoming in intersection.incomings:
                print(f"    - Incoming ID: {incoming.incoming_id}")
                print(f"      Incoming lanelets: {len(incoming.incoming_lanelets)} 个")
                print(f"      Successors left: {len(incoming.successors_left)} 个")
                print(f"      Successors straight: {len(incoming.successors_straight)} 个")
                print(f"      Successors right: {len(incoming.successors_right)} 个")
                
                # 收集点用于计算中心
                for lanelet_id in incoming.incoming_lanelets:
                    lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
                    if lanelet and len(lanelet.center_vertices) > 0:
                        all_points.append(lanelet.center_vertices[-1])
            
            # 计算中心位置
            if all_points:
                import numpy as np
                center = np.mean(np.array(all_points), axis=0)
                print(f"\n    中心位置（近似）: ({center[0]:.2f}, {center[1]:.2f})")
            
            # 检查是否有 crossings
            if hasattr(intersection, 'crossings') and intersection.crossings:
                print(f"    Crossings: {len(intersection.crossings)} 个")
                for crossing_id in intersection.crossings:
                    print(f"      - Crossing lanelet ID: {crossing_id}")
            else:
                print(f"    Crossings: 0 个")
    
    # 额外信息
    print("\n" + "-" * 70)
    print("场景统计信息:")
    print("-" * 70)
    print(f"Lanelet 总数: {len(scenario.lanelet_network.lanelets)}")
    print(f"Traffic sign 总数: {len(scenario.lanelet_network.traffic_signs)}")
    print(f"Traffic light 总数: {len(scenario.lanelet_network.traffic_lights)}")
    
except Exception as e:
    print(f"\n✗ 错误: {e}")
    import traceback
    traceback.print_exc()

