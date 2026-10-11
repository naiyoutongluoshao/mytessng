from PySide2.QtCore import *
from function import *
from Tessng import *
from config import *
import function
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import pandas as pd
from collections import defaultdict
from config import phase_lane_mapping, aggregate_time, capacity, phase2collector_dict

# / 自定义流量估算器类
class FlowEstimator:
    def __init__(self, max_history_length=5, default_flow=200, smoothing_factor=0.1):
        """
        初始化流量估算器
        :param max_history_length: 缓存的历史数据长度（用于移动平均）
        :param default_flow: 缺少数据时使用的默认流量值
        :param smoothing_factor: 用于指数平滑的因子，范围在 0 ~ 1 之间
        """
        self.max_history_length = max_history_length
        self.default_flow = default_flow
        self.smoothing_factor = smoothing_factor
        self.history_data = {}  # 用于存储每个相位的历史流量数据

    def add_flow_data(self, phase_id, flow_value):
        """
        将新的流量数据添加到历史记录中
        :param phase_id: 相位ID
        :param flow_value: 当前周期的流量数据
        """
        if phase_id not in self.history_data:
            self.history_data[phase_id] = []

        try:
            # 添加新的流量数据到历史记录
            self.history_data[phase_id].append(flow_value)

            # 如果历史记录超过最大长度，删除最旧的数据
            if len(self.history_data[phase_id]) > self.max_history_length:
                self.history_data[phase_id].pop(0)

        except Exception as e:
            print(f"[ERROR] 在添加流量数据时发生错误: {e}")

    def estimate_flow_based_on_history(self, phase_id, method="moving_average"):
        """
        根据历史数据估算相位的流量值
        :param phase_id: 相位ID
        :param method: 估算方法 ("moving_average", "exponential_smoothing", "weighted_average")
        :return: 估算的流量值
        """
        if phase_id in self.history_data and len(self.history_data[phase_id]) > 0:
            try:
                if method == "moving_average":
                    return self._moving_average(phase_id)
                elif method == "exponential_smoothing":
                    return self._exponential_smoothing(phase_id)
                elif method == "weighted_average":
                    return self._weighted_average(phase_id)
                else:
                    print(f"[WARNING] 不支持的方法: {method}，使用默认流量")
                    return self.default_flow
            except Exception as e:
                print(f"[ERROR] 在估算流量时发生错误: {e}")
                return self.default_flow
        else:
            # 如果没有足够的历史数据，返回默认流量值
            print(f"[WARNING] 缺少相位 {phase_id} 的历史数据，使用默认流量: {self.default_flow}")
            return self.default_flow

    def _moving_average(self, phase_id):
        """
        简单移动平均计算
        """
        estimated_flow = sum(self.history_data[phase_id]) / len(self.history_data[phase_id])
        print(f"[INFO] 使用移动平均估算相位 {phase_id} 的流量: {estimated_flow:.2f}")
        return estimated_flow

    def _exponential_smoothing(self, phase_id):
        """
        使用指数平滑计算流量
        """
        data = self.history_data[phase_id]
        smoothed_value = data[0]
        for value in data[1:]:
            smoothed_value = self.smoothing_factor * value + (1 - self.smoothing_factor) * smoothed_value
        print(f"[INFO] 使用指数平滑估算相位 {phase_id} 的流量: {smoothed_value:.2f}")
        return smoothed_value

    def _weighted_average(self, phase_id):
        """
        使用加权平均计算流量，最近的数据权重更高
        """
        data = self.history_data[phase_id]
        weights = [(i + 1) for i in range(len(data))]
        weighted_sum = sum(w * v for w, v in zip(weights, data))
        total_weight = sum(weights)
        estimated_flow = weighted_sum / total_weight
        print(f"[INFO] 使用加权平均估算相位 {phase_id} 的流量: {estimated_flow:.2f}")
        return estimated_flow

# / 自定义特种车俩处理类
class SpecialVehicleHandler:
    def __init__(self, simulator):
        """
        初始化特种车辆处理类
        :param simulator: 主仿真器实例
        """
        self.data_cololector = TrafficDataCollector
        self.simulator = simulator
        self.special_vehicle_info = None  # 存储当前特种车辆信息
        self.priority_signal_set = False
        self.last_debug_message_time = 0  # 上次打印时间
        self.debug_interval = 1000  # 控制打印频率，单位为毫秒
        self.printed_messages = {}  # 用于存储已打印消息的特种车辆ID

        self.right_turn_lanes = {
            2: [5],
            10: [40],  # 路段ID为10的右转车道ID为40
            11: [46],
            12: [53]
        }

    def is_right_turn_lane(self, current_link_id, lane_id):
        """
        判断是否为右转车道
        :param current_link_id: 当前路段ID
        :param lane_id: 当前车道ID
        :return: bool 表示是否为右转车道
        """
        return current_link_id in self.right_turn_lanes and lane_id in self.right_turn_lanes[current_link_id]

    def debug_print(self, message):
        """
        控制打印的辅助函数，限制过于频繁的打印
        """
        import time
        current_time = time.time() * 1000  # 获取当前时间（毫秒）
        if current_time - self.last_debug_message_time >= self.debug_interval:
            print(f"[DEBUG] {message}")
            self.last_debug_message_time = current_time

    def detect_special_vehicle(self, all_vehicles):
        """
        检测特种车辆并设置优先通行，排除右转车道
        """
        for vehicle in all_vehicles:
            if vehicle.vehicleTypeCode() in [9, 10]:  # 9、10是特种车辆类型编码
                vehicle_type = "消防车" if vehicle.vehicleTypeCode() == 9 else "救护车"
                vehicle_id = vehicle.id()

                # 检查是否已经打印过该车辆的消息
                if vehicle_id not in self.printed_messages:
                    self.printed_messages[vehicle_id] = True
                    self.debug_print(f"发现特种车辆：{vehicle_type}（ID: {vehicle_id}）")

                try:
                    lane = vehicle.lane()  # 获取车辆所在的车道
                    lane_id = lane.id() if lane else None
                    current_link = lane.link() if lane else None  # 从车道获取路段信息
                    current_link_id = current_link.id() if current_link else None

                    # 打印车辆位置信息（确保只打印一次）
                    if vehicle_id not in self.printed_messages:
                        print(
                            f"{vehicle_type}（ID: {vehicle_id}）位于车道ID: {lane_id if lane_id else '未知车道ID'}, 路段ID: {current_link_id if current_link_id else '未知路段ID'}")
                        self.printed_messages[vehicle_id] = True

                    # 检查是否为右转车道
                    if self.is_right_turn_lane(current_link_id, lane_id):
                        # 如果是右转车道，跳过优先通行处理
                        self.debug_print(
                            f"检测到 {vehicle_type}（ID: {vehicle_id}）在路段ID {current_link_id} 的右转车道，跳过优先通行处理。")
                        continue

                    # 根据 `dispatch_to_phase_mapping` 查找相位信息
                    found = False
                    for start_point, info in dispatch_to_phase_mapping.items():
                        if current_link_id in info["lanes"]:
                            phase_id = info["phase_id"]
                            self.simulator.debug_print(f"检测到特种车辆从路段ID: {current_link_id}，对应相位ID: {phase_id}")

                            # 设置优先通行相位
                            self.handle_special_vehicle_priority(start_point)

                            # 更新特种车辆信息
                            self.special_vehicle_info = {
                                "vehID": vehicle_id,
                                "startPointID": start_point,
                                "phaseID": phase_id
                            }
                            found = True
                            break
                    if not found:
                        self.simulator.debug_print(f"路段ID: {current_link_id} 未能找到对应的发车点ID，为右转车道，跳过优先处理。")

                except AttributeError as e:
                    self.debug_print(f"车辆ID: {vehicle_id} 未能获取到车道或路段信息，错误: {e}")
                    continue


    def is_in_phase_mapping(self, current_link_id):
        """
        判断当前路段ID是否在相位映射字典中
        :param current_link_id: 当前车辆所在的路段ID
        :return: bool 表示是否在字典中
        """
        for phase_lanes in phase_lane_mapping.values():
            for lane_info in phase_lanes:
                if current_link_id == lane_info["lane_id"]:
                    return True
        return False

    def set_priority_for_vehicle(self, vehicle, current_link_id):
        """
        设置特种车辆的优先通行
        :param vehicle: 特种车辆对象
        :param current_link_id: 车辆所在的路段ID
        """
        found = False
        for start_point, info in dispatch_to_phase_mapping.items():
            if current_link_id in info["lanes"]:
                phase_id = info["phase_id"]
                self.debug_print(f"检测到特种车辆（ID: {vehicle.id()}）从路段ID: {current_link_id}，对应相位ID: {phase_id}")

                # 处理特种车辆的优先通行
                self.handle_special_vehicle_priority(start_point)
                self.special_vehicle_info = {
                    "vehID": vehicle.id(),
                    "startPointID": start_point,
                    "phaseID": phase_id
                }
                found = True
                break

        if not found:
            self.debug_print(f"路段ID: {current_link_id} 未能找到对应的发车点ID，跳过优先处理。")

    def handle_special_vehicle_priority(self, start_point_id):
        """
        根据特种车辆的发车点ID处理优先通行
        """
        try:
            phase_id = self.simulator.determine_phase_by_start_point(start_point_id)

            if phase_id:
                if self.priority_signal_set and self.special_vehicle_info and \
                        self.special_vehicle_info["phaseID"] == phase_id:
                    self.debug_print(f"相位 {phase_id} 已设置为优先绿灯，无需重复设置。")
                    return

                iface = tessngIFace()
                netiface = iface.netInterface()
                group = netiface.signalGroups()[0]
                print(f"根据发车点 {start_point_id} 调整相位 {phase_id} 为优先通行")
                self.simulator.set_priority_signal(group, phase_id)
                self.priority_signal_set = True

                self.special_vehicle_info = {
                    "startPointID": start_point_id,
                    "phaseID": phase_id
                }
            else:
                print(f"未找到与发车点 {start_point_id} 对应的优先通行相位")
        except Exception as e:
            print(f"[ERROR] 在处理特种车辆优先时发生错误: {e}")

    def check_special_vehicle_passed(self, vehicle_id):
        """
        检查特种车辆是否已经通过信号灯。
        """
        iface = tessngIFace()
        simuiface = iface.simuInterface()

        all_vehicles = simuiface.allVehiStarted()

        # 检查当前运行的车辆中是否还存在特种车辆ID
        for vehicle in all_vehicles:
            if vehicle.id() == vehicle_id:
                return False  # 特种车辆还没有通过

        # 如果没有找到特种车辆，则表示它已经通过
        print(f"特种车辆 {vehicle_id} 已通过信号灯，恢复正常信控。")
        self.restore_normal_signal()

        # 清除特种车辆优先状态标志
        self.special_vehicle_info = None
        self.priority_signal_set = False
        return True

    def restore_normal_signal(self):
        """
        恢复正常的信控方案。
        """
        iface = tessngIFace()
        netiface = iface.netInterface()
        group = netiface.signalGroups()[0]

        print("恢复正常信控方案...")

        # 检查是否保存了先前的信号配时，如果有则恢复
        if hasattr(self, 'saved_phase_intervals') and self.saved_phase_intervals:
            phases = group.phases()
            for phase in phases:
                if phase.id() == self.special_vehicle_info.get("phaseID"):
                    phase.setColorList(self.saved_phase_intervals)
                    print(f"相位 {phase.id()} 已恢复为原始信控方案。")
        else:
            # 如果没有保存原有配时，调用默认恢复方法
            print("[WARNING] 没有保存原始配时，尝试恢复默认配置。")
            self.simulator.restore_default_signal_plan(group)

        # 清理状态
        self.special_vehicle_info = None
        self.priority_signal_set = False
        print("恢复了默认的信控方案")

# / 自定义数据采集类
class TrafficDataCollector:
    def __init__(self):
        """
        初始化交通数据采集类，设置必要的参数
        """
        # 数据采集阈值参数，具体值可根据实际需求调整
        self.queue_length_threshold = 50  # 米，排队长度阈值
        self.flow_threshold = 500  # 辆/小时，车辆流量阈值

    def get_queue_length(self, lane_id):
        """
        通过仿真接口获取指定车道的排队长度
        :param lane_id: 车道ID
        :return: 返回当前车道的排队长度
        """
        iface = tessngIFace()
        simuiface = iface.simuInterface()

        queue_length = 0
        try:
            # 获取车辆排队的统计数据
            queue_info = simuiface.getVehisQueueAggregated()
            for q in queue_info:
                if q.laneId == lane_id:
                    queue_length = q.avgQueueLength
        except Exception as e:
            print(f"获取车道 {lane_id} 排队长度时出错: {e}")

        return queue_length

    def get_flow(self, lane_id):
        """
        通过仿真接口获取指定车道的流量（辆/小时）
        :param lane_id: 车道ID
        :return: 返回当前车道的流量
        """
        iface = tessngIFace()
        simuiface = iface.simuInterface()

        flow = 0
        try:
            # 获取车道的流量信息
            collectors = simuiface.getVehisInfoAggregated()
            for collector in collectors:
                if collector.laneId == lane_id:
                    flow = collector.vehiCount / collector.aggregateTime * 3600  # 转换为小时流量
        except Exception as e:
            print(f"获取车道 {lane_id} 流量时出错: {e}")

        return flow

    def collect_traffic_data(self):
        """
        采集当前所有车道的流量和排队长度
        :return: 返回流量字典和缺失的采集器ID
        """
        iface = tessngIFace()
        simuiface = iface.simuInterface()

        # 获取所有采集器的数据
        collectors = simuiface.getVehisQueueAggregated()
        print(f"收集到的采集器信息: {collectors}")  # 打印采集器信息，帮助调试
        collector_flow_dict = {}
        missing_collectors = []

        for collector in collectors:
            collector_id = collector.collectorId
            veh_count = collector.vehiCount
            aggregate_time = collector.toTime - collector.fromTime

            if aggregate_time > 0:
                flow = veh_count / aggregate_time * 3600  # 转换为小时流量
                collector_flow_dict[collector_id] = flow
            else:
                missing_collectors.append(collector_id)

        # print(f"收集到的车道数据: {collector_flow_dict}")  # 调试输出
        # print(f"缺失的车道数据: {missing_collectors}")  # 调试输出

        return collector_flow_dict, missing_collectors

    def detect_congestion(self):
        """
        根据排队计数器的数据进行拥堵检测
        :return: 返回拥堵车道列表
        """
        iface = tessngIFace()
        simuiface = iface.simuInterface()

        congested_lanes = []
        lVehiQueueAggr = simuiface.getVehisQueueAggregated()

        if not lVehiQueueAggr:
            print("[ERROR] 无法获取任何排队计数器信息。检查计数器状态。")
            return congested_lanes

        # 处理排队长度数据，记录超过阈值的车道
        for vqAggr in lVehiQueueAggr:
            if vqAggr.avgQueueLength > self.queue_length_threshold:
                congested_lanes.append(vqAggr.counterId)

        return congested_lanes

    # def collect_traffic_data(self):
    #     """
    #     采集当前所有车道的流量和排队长度
    #     :return: 返回流量字典和缺失的采集器ID
    #     """
    #     iface = tessngIFace()
    #     simuiface = iface.simuInterface()
    #
    #     # 获取所有采集器的数据
    #     collectors = simuiface.getVehisInfoAggregated()
    #     collector_flow_dict = {}
    #     missing_collectors = []
    #
    #     for collector in collectors:
    #         collector_id = collector.collectorId
    #         veh_count = collector.vehiCount
    #         aggregate_time = collector.toTime - collector.fromTime
    #
    #         if aggregate_time > 0:
    #             flow = veh_count / aggregate_time * 3600  # 转换为小时流量
    #             collector_flow_dict[collector_id] = flow
    #         else:
    #             missing_collectors.append(collector_id)
    #
    #     # 确保返回的是一个字典类型
    #     return collector_flow_dict, missing_collectors

    # 获取道路信息
    def get_lane_info(self, counter_id):
        """
        根据计数器ID获取车道信息。
        """
        for lane_id, counters in lane_counter_dict.items():
            if counter_id in counters:
                return lane_id, counters[counter_id]
        return "Unknown Lane", {"车道序号": "Unknown", "方向": "Unknown", "车道方向": "Unknown", "位置": "Unknown"}

    def get_vehicle_density(self, lane_id):
        # 获取车道流量（可以先通过get_flow方法获取流量）
        flow = self.get_flow(lane_id)

        # 获取车道长度（单位 km），如果车道长度没有定义，默认返回1公里
        lane_length = self.lane_length_dict.get(lane_id, 1)  # 默认长度为 1 km

        # 防止车道长度为0的情况（虽然默认值已设置为1，但做更严格检查）
        if lane_length == 0:
            print(f"[WARNING] 车道 {lane_id} 的长度为零，使用默认值 1 km")
            lane_length = 1  # 防止除零错误，默认1公里

        # 车辆密度 = 流量 / 车道长度（辆/km）
        vehicle_density = flow / lane_length

        # 返回车辆密度
        return vehicle_density

# / 自定义拥堵检测类
class CongestionDetector:
    def __init__(self, queue_length_threshold=40, flow_threshold=500):
        """
        拥堵检测器
        :param queue_length_threshold: 排队长度阈值（米）
        :param flow_threshold: 车流量阈值（辆/小时）
        """
        self.queue_length_threshold = queue_length_threshold  # 排队长度阈值
        self.flow_threshold = flow_threshold  # 车流量阈值
        self.congested_lanes = []  # 记录拥堵的车道
        self.latest_congestion_status = False  # 记录最新的拥堵状态

    def detect_congestion(self):
        """
        根据排队计数器、流量等数据进行拥堵检测
        :return: 拥堵状态（True = 拥堵，False = 未拥堵）
        """
        iface = tessngIFace()
        simuiface = iface.simuInterface()

        # 获取排队计数器的数据
        lVehiQueueAggr = simuiface.getVehisQueueAggregated()
        if not lVehiQueueAggr:
            print("[ERROR] 无法获取任何排队计数器信息，请检查计数器状态。")
            return False

        self.congested_lanes = []  # 清空拥堵车道列表

        # 遍历所有的排队数据，分析拥堵情况
        for vqAggr in lVehiQueueAggr:
            counter_id = vqAggr.counterId
            avg_queue_length = vqAggr.avgQueueLength  # 平均排队长度（米）

            # 拥堵检测条件
            is_queue_long = avg_queue_length > self.queue_length_threshold

            if is_queue_long:
                self.congested_lanes.append(counter_id)

        # 打印当前拥堵情况
        self.latest_congestion_status = len(self.congested_lanes) > 0
        self.print_congestion_status()

        return self.latest_congestion_status

    def print_congestion_status(self):
        """
        输出当前的拥堵车道信息
        """
        if self.congested_lanes:
            message = "检测到拥堵车道："
            lane_messages = []

            for lane_id in self.congested_lanes:
                lane_info = self.get_lane_info(lane_id)
                if lane_info[0] != "Unknown Lane":
                    lane_messages.append(
                        f"{lane_info[1]['方向']} {lane_info[1]['车道方向']} （{lane_info[1]['位置']}）"
                    )
                else:
                    lane_messages.append(f"编号 {lane_id} 车道")

            print(message + ", ".join(lane_messages) + " 发生拥堵。")
        else:
            print("当前没有检测到拥堵情况。")

    def get_lane_info(self, counter_id):
        """
        根据计数器 ID 获取车道信息
        :param counter_id: 计数器 ID
        :return: 车道信息（车道编号，方向，位置等）
        """
        from config import lane_counter_dict  # 引入配置文件中的车道映射

        for lane_id, counters in lane_counter_dict.items():
            if counter_id in counters:
                return lane_id, counters[counter_id]
        return "Unknown Lane", {"车道序号": "Unknown", "方向": "Unknown", "车道方向": "Unknown", "位置": "Unknown"}

    def get_latest_status(self):
        """
        获取最新的拥堵状态
        :return: 拥堵状态（True/False）
        """
        return self.latest_congestion_status

# / 自定义交叉口评估类
class IntersectionEvaluator:
    def __init__(self):
        self.vehicle_travel_times = {}  # 存储车辆的行程时间 {veh_id: [起始时间, 结束时间]}
        self.vehicle_travel_distances = {}  # 存储车辆的行程距离 {veh_id: 路程}
        self.vehicle_delays = {}  # 存储车辆延误 {veh_id: 延误时间}
        self.phase_capacity = config.capacity  # 从config获取基础通行能力
        self.evaluation_interval = 300  # 每5分钟评估一次（与集计周期一致）

    def collect_metrics(self):
        """收集所有评价指标"""
        metrics = {
            'avg_travel_time': self._get_avg_travel_time(),
            'avg_travel_distance': self._get_avg_travel_distance(),
            'avg_vehicle_delay': self._get_avg_vehicle_delay()
        }
        return metrics

    def _get_avg_travel_time(self):
        """计算平均行程时间"""
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        current_time = simuiface.simuTimeIntervalWithAcceMutiples()

        travel_times = []

        # 更新行程时间
        for veh_id, times in self.vehicle_travel_times.items():
            veh = simuiface.getVehicle(veh_id)
            if veh is not None:
                if veh.isRunning():
                    self.vehicle_travel_times[veh_id][1] = current_time - self.vehicle_travel_times[veh_id][0]
                else:
                    travel_times.append(self.vehicle_travel_times[veh_id][1])

        avg_travel_time = sum(travel_times) / len(travel_times) if travel_times else 0
        return avg_travel_time

    def _get_avg_travel_distance(self):
        """计算平均行程距离"""
        total_distance = sum(self.vehicle_travel_distances.values())
        avg_distance = total_distance / len(self.vehicle_travel_distances) if self.vehicle_travel_distances else 0
        return avg_distance

    def _get_avg_vehicle_delay(self):
        """计算车均延误"""
        delays = []

        # 更新车辆的延误
        for veh_id, delay in self.vehicle_delays.items():
            veh = tessngIFace().simuInterface().getVehicle(veh_id)
            if veh is not None:
                if veh.isRunning():
                    self.vehicle_delays[veh_id] = delay
                else:
                    delays.append(delay)

        avg_delay = sum(delays) / len(delays) if delays else 0
        return avg_delay

    def _calculate_vehicle_delay(self, vehicle, ideal_time):
        """计算每辆车的延误时间"""
        actual_time = vehicle.actualPassTime()  # 获取车辆实际通过时间（假设存在）
        if actual_time is not None:
            delay = actual_time - ideal_time
            self.vehicle_delays[vehicle.id()] = delay


# 用户插件子类，代表用户自定义与仿真相关的实现逻辑，继承自PyCustomerSimulator
# 多重继承中的父类QObject，在此目的是要能够自定义信号signlRunInfo
class MySimulator(QObject, PyCustomerSimulator):
    signalRunInfo = Signal(str)
    forStopSimu = Signal()
    forReStartSimu = Signal()

    def __init__(self, net):
        # 初始化父类和必要的属性
        self.all_epoch_rewards=[]
        QObject.__init__(self)

        PyCustomerSimulator.__init__(self)
        self.mNetInf = net

        # 初始化数据采集器
        self.data_collector = TrafficDataCollector()

        # 仿真参数初始化
        self.Max_Simu_Count = 10  # 最大仿真次数
        self.Simu_Accuracy = 5  # 仿真精度
        self.Simu_Speed = 100  # 仿真倍数
        self.current_simu_count = 0  # 当前仿真次数
        self.simu_start_time = None  # 仿真开始时间

        # 设置日志记录器
        self.logger = setup_logger("Log\\MySimulator.log")

        # # 拥堵检测阈值参数，可以根据实际情况调整
        self.queue_length_threshold = 50  # 米，排队长度阈值
        self.speed_threshold = 10  # km/h，车辆速度阈值
        self.density_threshold = 20  # 辆/km，车辆密度阈值
        self.flow_threshold = 500  # 辆/小时，车辆流量阈值
        self.last_debug_message_time = 0  # 记录上一次打印的时间
        self.debug_interval = 10000  # 设定时间间隔（毫秒），例如每秒打印一次
        self.special_vehicle_info = None
        self.default_signal_plan_saved = True
        self.flow_estimator = FlowEstimator()  # 流量估算器
        self.special_vehicle_handler = SpecialVehicleHandler(self) # 特种车辆处理
        # 创建拥堵检测器对象
        self.congestion_detector = CongestionDetector()

        # self.monitor_ui = MonitorUI(self)
        # self.monitor_ui.show()  # 启动可视化监控界面
        self.evaluator = IntersectionEvaluator()

        print("默认信控方案已初始化")

    def afterVehicleGenerated(self, vehi):
        self.evaluator.update_vehicle_entry_time(vehi)
        return super().afterVehicleGenerated(vehi)

    # 初始化仿真设置
    def ref_beforeStart(self, ref_keepOn):
        iface = tessngIFace()
        simuiface = iface.simuInterface()

        # 设置仿真精度和倍速
        simuiface.setSimuAccuracy(self.Simu_Accuracy)
        simuiface.setAcceMultiples(self.Simu_Speed)

        # 记录仿真开始时间
        self.simu_start_time = time.time()
        self.current_simu_count += 1
        self.logger.info(f"第【{self.current_simu_count}】次仿真开始")

    # 获取发车点信息
    def determine_phase_by_start_point(self, start_point_id):
        """
        根据发车点ID确定对应的信号相位ID
        """
        # 使用传入的 start_point_id 进行查找
        if not hasattr(self, 'printed_start_points'):
            self.printed_start_points = set()

        if str(start_point_id) in dispatch_to_phase_mapping:
            phase_info = dispatch_to_phase_mapping[str(start_point_id)]
            phase_id = phase_info["phase_id"]

            # 仅在未打印过时进行打印
            if start_point_id not in self.printed_start_points:
                print(f"发车点 {start_point_id} 的相位ID为: {phase_id}")
                self.printed_start_points.add(start_point_id)

            return phase_id
        else:
            # 仅在未打印过时进行打印
            if start_point_id not in self.printed_start_points:
                print(f"无法确定发车点 {start_point_id} 对应的相位ID")
                self.printed_start_points.add(start_point_id)

            return None

    # 恢复默认信控方案
    def restore_default_signal_plan(self, group):
        """
        从 self.default_signal_phases 中恢复默认信控方案。
        """
        if not self.default_signal_phases:
            print("没有默认信控方案可以恢复。")
            return

        # 直接从 self.default_signal_phases 中恢复
        for phase in group.phases():
            phase_id = phase.id()
            if phase_id in self.default_signal_phases:
                default_color_list = self.default_signal_phases[phase_id]
                phase.setColorList(default_color_list)
                print(f"相位 {phase_id} 已恢复为默认信控方案")
            else:
                print(f"未找到相位 {phase_id} 的默认信控配置")

        print("恢复了默认的信控方案")

    # 恢复正常的信控方案
    def restore_normal_signal(self):
        """
        恢复正常的信控方案。
        """
        iface = tessngIFace()
        netiface = iface.netInterface()
        group = netiface.signalGroups()[0]

        # 恢复之前的正常信控方案
        print("恢复正常信控方案...")
        self.restore_default_signal_plan(group)

        # 清空特种车辆信息
        self.special_vehicle_info = None  # 在恢复信控方案时，确保清除信息

    # 控制打印频率的辅助函数
    def debug_print(self, message):
        """
        控制打印的辅助函数，限制过于频繁的打印
        """
        current_time = time.time() * 1000  # 获取当前时间（毫秒）
        if current_time - self.last_debug_message_time >= self.debug_interval:
            print(f"[DEBUG] {message}")
            self.last_debug_message_time = current_time

    def get_lane_info(self, counter_id):
        """
        根据计数器ID获取车道信息。
        """
        for lane_id, counters in lane_counter_dict.items():
            if counter_id in counters:
                return lane_id, counters[counter_id]
        return "Unknown Lane", {"车道序号": "Unknown", "方向": "Unknown", "车道方向": "Unknown", "位置": "Unknown"}

    # 优化的拥堵检测模块
    def detect_congestion(self):
        """
        根据排队计数器的数据进行拥堵检测，每次调用都打印当前拥堵情况
        """
        iface = tessngIFace()
        simuiface = iface.simuInterface()

        # 获取排队计数器的数据
        # print("尝试从排队计数器获取数据...")
        lVehiQueueAggr = simuiface.getVehisQueueAggregated()

        # 检查是否成功获取排队计数器数据
        if not lVehiQueueAggr:
            print("[ERROR] 无法获取任何排队计数器信息。检查计数器状态。")
            return

        # 打印收集到的排队计数器数据
        # print("=== 收集到的排队计数器数据 ===")
        # for vqAggr in lVehiQueueAggr:
        #     print(f"计数器 ID: {vqAggr.counterId}, 平均排队长度: {vqAggr.avgQueueLength} 米, "
        #           f"开始时间: {vqAggr.fromTime}, 结束时间: {vqAggr.toTime}")

        # 处理排队长度数据，记录超过阈值的车道
        congested_lanes = []
        for vqAggr in lVehiQueueAggr:
            if vqAggr.avgQueueLength > self.queue_length_threshold:
                congested_lanes.append(vqAggr.counterId)

        # 打印当前的拥堵车道情况
        if congested_lanes:
            message = "检测到新的拥堵车道："
            lane_messages = []

            for lane_id in congested_lanes:
                lane_info = self.get_lane_info(lane_id)

                if lane_info[0] != "Unknown Lane":
                    lane_messages.append(
                        f"{lane_info[1]['方向']} {lane_info[1]['车道方向']} （{lane_info[1]['位置']}）"
                    )
                else:
                    lane_messages.append(f"编号 {lane_id} 车道")

            # 打印综合信息
            print(message + ", ".join(lane_messages) + " 出现拥堵。")
        else:
            print("当前没有检测到拥堵情况。")

    # 设置优先信号
    # 为特种车辆设置优先信号
    def set_priority_signal(self, group, phase_id):
        """
        为特种车辆设置优先信号，将对应相位设置为绿灯。
        """
        phases = group.phases()
        for phase in phases:
            if phase.id() == phase_id:
                print(f"为相位 {phase_id} 设置优先绿灯...")
                new_color_lst = [
                    Online.ColorInterval('红', 0),  # 红灯时长 0
                    Online.ColorInterval('绿', 60),  # 绿灯时长 60
                    Online.ColorInterval('黄', 3)  # 黄灯时长 3
                ]
                phase.setColorList(new_color_lst)
                print(f"相位 {phase_id} 已设置为优先绿灯。")

    # 每轮仿真之后
    def afterStop(self):
        spend_time = time.time() - self.simu_start_time
        self.logger.info(f"第【{self.current_simu_count}】次仿真运行结束！当前已经用时为{spend_time:.1f}秒\\n")

        # 判断当前仿真次数是否达到最大仿真次数
        if self.current_simu_count < self.Max_Simu_Count:
            # 再次开启仿真
            self.forReStartSimu.emit()
            return

        print(self.all_epoch_rewards)

        spend_time = time.time() - self.simu_start_time
        self.logger.info(f"【{self.Max_Simu_Count}】轮仿真已经全部运行结束，总用时为{spend_time:.1f}秒!!!")
        print('评价报告保存到SimuResult\机动车交叉口.tess下')

    # 仿真每一步结束后的处理逻辑
    def afterOneStep(self):
        iface = tessngIFace()
        simuiface = iface.simuInterface()
        netiface = iface.netInterface()
        simuTime = simuiface.simuTimeIntervalWithAcceMutiples()

        # 获取当前仿真中的所有车辆
        all_vehicles = simuiface.allVehiStarted()
        self.debug_print(f"当前仿真中的车辆数量: {len(all_vehicles)}")

        # 处理特种车辆检测与优先通行
        self.special_vehicle_handler.detect_special_vehicle(all_vehicles)

        # 检查特种车辆是否通过
        if self.special_vehicle_info and 'vehID' in self.special_vehicle_info:
            vehicle_passed = self.check_special_vehicle_passed(self.special_vehicle_info["vehID"])
            if vehicle_passed:
                passed_time = simuiface.simuTimeIntervalWithAcceMutiples()
                if self.special_vehicle_info:
                    print(f"[{passed_time}] 特种车辆 {self.special_vehicle_info['vehID']} 已通过，恢复正常信控。")

                # 确保状态清除，以避免重复操作
                self.special_vehicle_info = None
                self.priority_signal_set = False

        # 确保在特定时间间隔运行 Webster 算法
        if simuTime / (config.aggregate_time * 1000) > 1 and simuTime % (config.aggregate_time * 1000) == 1000:
            print(f"[{simuTime}ms] 达到 Webster 算法调用时间间隔...")

            if self.congestion_detector.detect_congestion():
                print("当前路口发生拥堵")
                self.congestion_detector.print_congestion_status()
            else:
                print("当前路口没有发生拥堵")

            # ========== 数据采集阶段 ==========
            # print("尝试从采集器获取数据...")
            collectors = simuiface.getVehisInfoAggregated()

            # 检查采集器数据是否成功获取
            if not collectors:
                print("[ERROR] 无法获取任何采集器信息。检查采集器状态。")
                return

            # print("=== 采集到的采集器数据 ===")
            collector_flow_dict = {}
            missing_collectors = []
            # print(f"收集到的车道数据111: {collector_flow_dict}")  # 调试输出
            # print(f"缺失的车道数据111: {missing_collectors}")  # 调试输出

            # 遍历每一个采集器的数据
            for collector in collectors:
                collector_id = collector.collectorId
                veh_count = collector.vehiCount
                aggregate_time = collector.toTime - collector.fromTime

                # print(f"采集器 ID: {collector_id}, 车辆计数: {veh_count}, "
                #       f"数据起始时间: {collector.fromTime}, 数据结束时间: {collector.toTime}, "
                #       f"采集间隔: {aggregate_time} 秒")

                # 确保 `aggregate_time` 大于 0
                if aggregate_time > 0:
                    flow = veh_count / aggregate_time * 3600  # 扩大为小时流量
                    collector_flow_dict[collector_id] = flow
                else:
                    missing_collectors.append(collector_id)

                # 打印详细的采集器流量字典
                if collector_flow_dict:
                    # print(f"成功获取的流量数据: {collector_flow_dict}")
                    print(f"成功获取流量数据 - 数据采集器器ID: {collector_id}, 流量: {flow} 辆/小时")
                else:
                    print("[WARNING] 所有采集器数据均无效，请检查仿真系统配置。")

            # 如果有缺失的数据，记录并输出警告
            if missing_collectors:
                print(f"[WARNING] 以下采集器未能提供有效数据: {missing_collectors}")


            # 获取当前仿真时间排队计数器计数的车辆排队信息
            lVehiQueue = simuiface.getVehisQueueCounted()
            if len(lVehiQueue) > 0:
                # 使用列表推导式来创建一个包含(counterId, queueLength)元组的列表
                queue_info_list = [(vq.counterId, vq.queueLength) for vq in lVehiQueue]

                # 格式化输出
                print("车辆排队计数器计数：")
                for counter_id, queue_length in queue_info_list:
                    print(f"排队计数器ID: {counter_id}, 排队长度: {queue_length}")

            # 获取最近集计时间段内排队计数器集计数据
            lVehiQueueAggr = simuiface.getVehisQueueAggregated()
            if len(lVehiQueueAggr) > 0:
               print("车辆排队集计数据：", [(vqAggr.counterId, vqAggr.avgQueueLength) for vqAggr in lVehiQueueAggr])

            # 获取当前仿真时间行程时间检测器完成的行程时间检测信息
            lVehiTravel = simuiface.getVehisTravelDetected()
            if len(lVehiTravel) > 0:
               print("车辆行程时间检测信息：", [(vtrav.detectedId, vtrav.travelDistance) for vtrav in lVehiTravel])

            # 获取最近集计时间段内行程时间检测器集计数据
            lVehiTravAggr = simuiface.getVehisTravelAggregated()
            if len(lVehiTravAggr) > 0:
               print("车辆行程时间集计数据：", [(vTravAggr.detectedId, vTravAggr.vehiCount, vTravAggr.avgTravelDistance) for vTravAggr in lVehiTravAggr])

            # ========== 后续处理， phase_flow_dict 和信号优化 ==========
            phase_flow_dict = {}
            for phase_id, collector_lst in config.phase2collector_dict.items():
                valid_flows = [collector_flow_dict.get(collector_id, 0) for collector_id in collector_lst if
                               collector_id in collector_flow_dict]
                if valid_flows:
                    avg_flow = sum(valid_flows) / len(valid_flows)  # 计算平均流量
                    phase_flow_dict[phase_id] = avg_flow
                else:
                    estimated_flow = self.flow_estimator.estimate_flow_based_on_history(phase_id)
                    phase_flow_dict[phase_id] = estimated_flow
                    print(f"[INFO] 使用移动平均估算相位 {phase_id} 的流量: {estimated_flow:.2f} 辆/小时")

            # 打印生成的 phase_flow_dict，增加描述和格式化输出
            print("\n根据各个相位的检测器数据计算得到的平均交通流量为：")
            for phase_id, flow_rate in phase_flow_dict.items():
                print(f"相位 {phase_id}: 平均流量为 {flow_rate:.2f} 辆/小时")
            # 检查是否有缺少的流量数据
            missing_phases = [phase_id for phase_id in config.phase2collector_dict.keys() if
                              phase_id not in phase_flow_dict]
            if missing_phases:
                print(f"[ERROR] 以下相位缺少流量数据: {missing_phases}")
                print("维持当前信控方案，不恢复默认方案以免误操作。")
                return

            # 调用 Webster 配时优化逻辑
            group = netiface.signalGroups()[0]
            period_time = group.periodTime()
            phase_interval_dict = {}

            for phase in group.phases():
                start_time = 0
                green_interval = 0
                yellow_interval = 0
                for color_interval in phase.listColor():
                    if color_interval.color == '红':
                        start_time += color_interval.interval
                    elif color_interval.color == '绿':
                        green_interval = color_interval.interval
                    elif color_interval.color == '黄':
                        yellow_interval = color_interval.interval

                phase_interval_dict[phase.id()] = {
                    'phase_id': phase.id(),
                    'start_time': start_time,
                    'green_interval': green_interval,
                    'yellow_interval': yellow_interval,
                    'all_red_interval': 0  # Assuming this is intended to be 0 or calculated elsewhere
                }

            # 打印生成的 phase_interval_dict，增加描述和格式化输出
            print("\n根据信号灯组计算得到的各相位时间间隔为：")
            for phase_info in phase_interval_dict.values():
                print(f"相位 {phase_info['phase_id']}:")
                print(f"  - 开始时间（从周期开始到该相位绿灯亮起的时间）: {phase_info['start_time']} 秒")
                print(f"  - 绿灯持续时间: {phase_info['green_interval']} 秒")
                print(f"  - 黄灯持续时间: {phase_info['yellow_interval']} 秒")
                print(f"  - 全红时间（所有方向均为红灯的时间间隔）: {phase_info['all_red_interval']} 秒\n")

            # 计算韦伯斯特信号配时
            try:
                # 调用 Webster 算法计算新的周期时间和相位时间间隔
                new_period_time, new_phase_interval_dict = function.webster(
                    period_time, phase_interval_dict, phase_flow_dict, flow_limit=1.0  # 设置更高的流量上限
                )

                # 打印生成的 new_period_time 和 new_phase_interval_dict，增加描述和格式化输出
                print("\n根据Webster算法优化后的结果为：")
                print(f"新周期时间: {new_period_time} 秒")

                print("各相位的时间间隔调整为：")
                for phase_id, interval_info in new_phase_interval_dict.items():
                    print(f"相位 {phase_id}:")
                    print(f"  - 开始时间（从周期开始到该相位绿灯亮起的时间）: {interval_info['start_time']} 秒")
                    print(f"  - 绿灯持续时间: {interval_info['green_interval']} 秒")
                    print(f"  - 黄灯持续时间: {interval_info['yellow_interval']} 秒")
                    print(f"  - 全红时间（所有方向均为红灯的时间间隔）: {interval_info['all_red_interval']} 秒\n")

                # 修改仿真中的信控方案
                group.setPeriodTime(new_period_time)
                for phase in group.phases():
                    new_phase = new_phase_interval_dict[phase.id()]
                    new_color_lst = [
                        Online.ColorInterval('红', new_phase.get('start_time')),
                        Online.ColorInterval('绿', new_phase.get('green_interval')),
                        Online.ColorInterval('黄', new_phase.get('yellow_interval')),
                        Online.ColorInterval('红', max((new_period_time - new_phase.get('start_time') -
                                                       new_phase.get('green_interval') -
                                                       new_phase.get('yellow_interval')), 0))
                    ]
                    phase.setColorList(new_color_lst)

                print('下发新的信控方案')

            except Exception as e:
                print(f"Webster算法错误: {e}")
                print("维持当前信控方案，不恢复默认方案以免误操作。")