import math
import config
import os
import logging
import numpy as np

# 记录日志
def setup_logger(log_file='example.log', level=logging.INFO):
    # 创建日志记录器对象
    logger = logging.getLogger(__name__)
    # 清空已存在的处理程序和过滤器，以防止重复添加
    logger.handlers = []
    logger.filters = []
    # 配置日志记录器，将日志信息写入文件，并设置日志级别
    logging.basicConfig(
        filename=log_file,
        level=level,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    # 创建一个处理程序，输出到控制台
    handler = logging.StreamHandler()
    handler.setLevel(level)  # 设置控制台处理程序的日志级别
    # 创建一个格式化器，定义控制台输出的日志信息格式
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)  # 为处理程序设置格式化器
    # 将控制台处理程序添加到日志记录器
    logger.addHandler(handler)
    # 返回配置好的日志记录器对象
    return logger

# def webster(period_time, phase_interval_dict, phase_flow_dict, min_green_time=5, max_green_time=60, flow_limit=1.0):
#     """
#     Webster 算法改进版
#     """
#     # 相位数
#     k = len(phase_interval_dict)
#
#     # 全红时间
#     all_red_interval_lst = [content['all_red_interval'] for key, content in phase_interval_dict.items()]
#
#     # 黄灯时间
#     yellow_interval_lst = [content['yellow_interval'] for key, content in phase_interval_dict.items()]
#
#     # 计算信号总损失时间L
#     L = sum(all_red_interval_lst) + sum(yellow_interval_lst)
#
#     # 各相位车道数和流量
#     flow_lst = [phase_flow_dict[phase] for phase in phase_interval_dict.keys()]
#
#     # 计算流量比
#     y_lst = [i / config.capacity for i in flow_lst]
#     total_y = sum(y_lst)
#
#     # 检查流量比是否超过合理范围
#     if total_y > 0.85:
#         print("[WARNING] 总流量比超出范围，调整至上限0.85进行计算。")
#         total_y = 0.85
#
#     # 计算交叉口周期C（向上取整）
#     c0 = period_time
#     c = math.ceil(L / (1 - total_y))
#     c = min(max(c, c0 * 0.5), c0 * 2)  # 周期取值范围
#     new_period_time = c
#
#     # 计算总有效绿灯时间G_e（向下取整）
#     G_e = c - math.floor(L)
#
#     # 计算有效绿灯时间g_e
#     green_interval_lst = [max(min_green_time, math.floor((y_lst[i] / total_y) * G_e)) for i in range(k)]
#
#     # 调整有效绿灯时间g_e，保证其值之和等于总有效绿灯时间G_e
#     green_diff = G_e - sum(green_interval_lst)
#     if green_diff > 0:
#         # 尝试分配多余的绿灯时间
#         for i in range(k):
#             if green_interval_lst[i] < max_green_time:
#                 extra_time = min(green_diff, max_green_time - green_interval_lst[i])
#                 green_interval_lst[i] += extra_time
#                 green_diff -= extra_time
#                 if green_diff <= 0:
#                     break
#
#     # 新的信号灯组
#     new_phase_interval_dict = {}
#     for phase, content in phase_interval_dict.items():
#         i = list(phase_interval_dict.keys()).index(phase)
#         # 保证绿灯时间不小于最小绿灯时间
#         green_time = max(green_interval_lst[i], min_green_time)
#         new_phase_interval_dict[phase] = {
#             'phase_id': phase,
#             'start_time': sum(green_interval_lst[:i]) + sum(yellow_interval_lst[:i]),
#             'green_interval': green_time,
#             'yellow_interval': yellow_interval_lst[i],
#             'all_red_interval': all_red_interval_lst[i]
#         }
#
#     return new_period_time, new_phase_interval_dict


def webster(period_time, phase_interval_dict, phase_flow_dict, min_green_time=5, max_green_time=60, flow_limit=1.0):
    """
    修正后的 Webster 信号配时优化算法
    """
    # 相位数
    k = len(phase_interval_dict)

    # 计算信号损失时间 L（所有相位的全红时间 + 黄灯时间）
    all_red_interval_lst = [content['all_red_interval'] for content in phase_interval_dict.values()]
    yellow_interval_lst = [content['yellow_interval'] for content in phase_interval_dict.values()]
    L = sum(all_red_interval_lst) + sum(yellow_interval_lst)

    # 计算每个相位的流量比 y = q / s
    flow_lst = [phase_flow_dict.get(phase, 0) for phase in phase_interval_dict.keys()]
    y_lst = [i / config.capacity for i in flow_lst]

    # 计算总流量比 Y
    total_y = sum(y_lst)

    # 约束流量比最大值 0.85
    if total_y > 0.85:
        print("[WARNING] 总流量比超出范围，调整至上限 0.85 进行计算。")
        total_y = 0.85

    # 计算新的周期时长 C
    c0 = period_time  # 原始周期
    c = math.ceil((1.5 * L + 5) / (1 - total_y))  # Webster 计算周期
    new_period_time = min(max(c, c0 * 0.5), c0 * 2)  # 限制周期变化范围

    # 计算总有效绿灯时间 G_e
    G_e = new_period_time - L

    # 计算初步的绿灯时间分配
    green_interval_lst = [
        max(min_green_time, round((y_lst[i] / total_y) * G_e)) if total_y > 0 else min_green_time
        for i in range(k)
    ]

    # 调整绿灯时间，使得总和不超过 G_e
    green_total = sum(green_interval_lst)
    if green_total > G_e:
        scale_factor = G_e / green_total
        green_interval_lst = [max(min_green_time, round(g * scale_factor)) for g in green_interval_lst]

    # 确保调整后的绿灯时间总和不超过 G_e
    while sum(green_interval_lst) > G_e:
        max_index = green_interval_lst.index(max(green_interval_lst))
        green_interval_lst[max_index] -= 1  # 逐步减少最长的绿灯时间

    # **重新计算 start_time 并存入字典**
    start_time = 0
    new_phase_interval_dict = {}
    for i, phase in enumerate(phase_interval_dict.keys()):
        green_time = green_interval_lst[i]
        yellow_time = yellow_interval_lst[i]
        all_red_time = all_red_interval_lst[i]

        # 确保 `start_time + phase_duration` 不会超过 `new_period_time`
        phase_duration = green_time + yellow_time + all_red_time
        if start_time + phase_duration > new_period_time:
            green_time = max(min_green_time, new_period_time - start_time - yellow_time - all_red_time)

        new_phase_interval_dict[phase] = {
            'phase_id': phase,
            'start_time': start_time,
            'green_interval': green_time,
            'yellow_interval': yellow_time,
            'all_red_interval': all_red_time
        }
        start_time += phase_duration

    return new_period_time, new_phase_interval_dict


