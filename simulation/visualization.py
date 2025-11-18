import os
import numpy as np
import json
from pathlib import Path
import matplotlib
matplotlib.use('TkAgg')  # 或者 'Qt5Agg'
import matplotlib.pyplot as plt
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import MPDrawParams

def render_gui(scenario):
    """可视化包含车辆的场景"""
    rnd = MPRenderer()
    # 绘制原始场景
    scenario.draw(rnd)
    rnd.render(show=True)
    # 保持窗口打开
    plt.show(block=True)
    return rnd

def create_vehicle_animation(scenario,time_step=50,dt=100,delta_time_steps=1,output_path='outputs/gifs/vehicle_animation.gif'):
    """使用官方方法创建车辆动画GIF"""
    # 创建渲染器
    rnd = MPRenderer()
    scenario.draw(rnd)
    # 设置绘制参数
    draw_params = MPDrawParams()
    draw_params.time_end = time_step  # 动画时长
    draw_params.dynamic_obstacle.show_label = False
    draw_params.dynamic_obstacle.draw_icon = True
    draw_params.dynamic_obstacle.draw_shape = True

    
    # 使用官方create_video方法，传入场景列表
    rnd.create_video(
        obj_lists=[scenario], 
        file_path=output_path, 
        draw_params=draw_params,
        dt=dt,  # 每帧之间的时间间隔（毫秒）
    delta_time_steps=delta_time_steps,  # 每多少时间步绘制一帧
    )