#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32机器人控制器数据监控器
支持实时显示各种传感器和状态数据
基于RRC通信协议的完整数据监控可视化工具
"""

import serial
import time
import struct
import threading
from enum import Enum
from datetime import datetime
import os
import sys

# 尝试导入可选的可视化库
try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext
    GUI_AVAILABLE = True
except ImportError:
    GUI_AVAILABLE = False
    print("警告: tkinter不可用，将使用终端显示模式")

# --- CRC-8校验算法 ---
crc8_table = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]

def checksum_crc8(data):
    check = 0
    for b in data:
        check = crc8_table[check ^ b]
    return check & 0xFF

# --- 协议解析器 ---
class ParseState(Enum):
    WAIT_HEADER_1 = 1
    WAIT_HEADER_2 = 2
    WAIT_FUNCTION = 3
    WAIT_LENGTH = 4
    READ_DATA = 5
    READ_CHECKSUM = 6

class RrcProtocolParser:
    def __init__(self):
        self._reset()

    def _reset(self):
        self.state = ParseState.WAIT_HEADER_1
        self.function_code = 0
        self.data_length = 0
        self.data_buffer = bytearray()
        
    def calculate_checksum(self):
        crc_data = bytearray([self.function_code, self.data_length])
        crc_data.extend(self.data_buffer)
        return checksum_crc8(crc_data)

    def parse_byte(self, byte_in):
        if self.state == ParseState.WAIT_HEADER_1:
            if byte_in == 0xAA: 
                self.state = ParseState.WAIT_HEADER_2
        elif self.state == ParseState.WAIT_HEADER_2:
            if byte_in == 0x55: 
                self.state = ParseState.WAIT_FUNCTION
            else: 
                self._reset()
        elif self.state == ParseState.WAIT_FUNCTION:
            self.function_code = byte_in
            self.state = ParseState.WAIT_LENGTH
        elif self.state == ParseState.WAIT_LENGTH:
            self.data_length = byte_in
            self.data_buffer = bytearray()
            if self.data_length == 0: 
                self.state = ParseState.READ_CHECKSUM
            else: 
                self.state = ParseState.READ_DATA
        elif self.state == ParseState.READ_DATA:
            self.data_buffer.append(byte_in)
            if len(self.data_buffer) == self.data_length: 
                self.state = ParseState.READ_CHECKSUM
        elif self.state == ParseState.READ_CHECKSUM:
            expected_checksum = self.calculate_checksum()
            received_checksum = byte_in
            packet = None
            if expected_checksum == received_checksum:
                packet = {
                    "function_code": self.function_code, 
                    "data_length": self.data_length, 
                    "data": self.data_buffer,
                    "timestamp": time.time()
                }
            else:
                raw_packet_hex = f"AA 55 {self.function_code:02X} {self.data_length:02X} {' '.join(f'{b:02X}' for b in self.data_buffer)} {received_checksum:02X}"
                print(f"[CRC错误] 期望: {expected_checksum:02X}, 接收: {received_checksum:02X}, 数据: [{raw_packet_hex}]")
            self._reset()
            return packet
        return None

# --- 数据存储类 ---
class RobotDataStore:
    def __init__(self):
        self.reset_all_data()
        
    def reset_all_data(self):
        # 系统信息
        self.system_data = {
            'battery_voltage': 0.0,
            'last_update': None
        }
        
        # 编码器数据 (批量)
        self.encoder_data = {
            'motor_0': {'id': 0, 'counter': 0, 'rps': 0.0, 'rpm': 0.0},
            'motor_1': {'id': 1, 'counter': 0, 'rps': 0.0, 'rpm': 0.0},
            'motor_2': {'id': 2, 'counter': 0, 'rps': 0.0, 'rpm': 0.0},
            'motor_3': {'id': 3, 'counter': 0, 'rps': 0.0, 'rpm': 0.0},
            'last_update': None
        }
        
        # IMU数据
        self.imu_data = {
            'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'last_update': None
        }
        
        # 手柄数据
        self.gamepad_data = {
            'buttons': 0,
            'hat': 0,
            'left_stick': {'x': 0, 'y': 0},
            'right_stick': {'x': 0, 'y': 0},
            'last_update': None
        }
        
        # 按键事件
        self.key_data = {
            'key_id': 0,
            'event': 0,
            'event_name': '',
            'last_update': None
        }
        
        # SBUS遥控器
        self.sbus_data = {
            'channels': [0] * 16,
            'ch17': 0,
            'ch18': 0,
            'signal_loss': False,
            'fail_safe': False,
            'last_update': None
        }
        
        # 总线舵机信息
        self.bus_servo_data = {
            'servo_data': [0] * 7,
            'last_update': None
        }
        
        # 统计信息
        self.stats = {
            'total_packets': 0,
            'valid_packets': 0,
            'crc_errors': 0,
            'packet_counts': {},
            'start_time': time.time()
        }

# --- 数据解析器 ---
class DataPacketHandler:
    def __init__(self, data_store):
        self.data_store = data_store
        
    def handle_packet(self, packet):
        func_code = packet['function_code']
        data = packet['data']
        timestamp = packet['timestamp']
        
        # 更新统计信息
        self.data_store.stats['total_packets'] += 1
        self.data_store.stats['valid_packets'] += 1
        
        if func_code not in self.data_store.stats['packet_counts']:
            self.data_store.stats['packet_counts'][func_code] = 0
        self.data_store.stats['packet_counts'][func_code] += 1
        
        # 调试信息：只对特定数据包类型显示（减少输出）
        if func_code in [0x09, 0x0B] and self.data_store.stats['packet_counts'][func_code] % 50 == 1:  # 每50个包显示一次
            func_names = {
                0x00: "系统信息", 0x06: "按键事件", 0x07: "IMU数据", 0x08: "总线舵机",
                0x09: "IMU数据", 0x0A: "手柄数据", 0x0B: "编码器数据", 0x0C: "OLED控制"
            }
            func_name = func_names.get(func_code, f"未知(0x{func_code:02X})")
            print(f"[DEBUG] 接收到: {func_name} (0x{func_code:02X}), 长度: {len(data)}, 总计: {self.data_store.stats['packet_counts'][func_code]}")
        
        # 根据功能码解析数据
        if func_code == 0x00:  # 系统信息
            self._parse_system_data(data, timestamp)
        elif func_code == 0x06:  # 按键事件
            self._parse_key_event(data, timestamp)
        elif func_code == 0x07:  # IMU数据 (24字节)
            self._parse_imu_data(data, timestamp)
        elif func_code == 0x08:  # 总线舵机信息
            self._parse_bus_servo_info(data, timestamp)
        elif func_code == 0x09:  # IMU数据
            self._parse_imu_data(data, timestamp)
        elif func_code == 0x0A:  # 手柄数据
            self._parse_gamepad_data(data, timestamp)
        elif func_code == 0x0B:  # 编码器数据 (37字节)
            self._parse_encoder_data(data, timestamp)
            self._parse_sbus_data(data, timestamp)
            
    def _parse_system_data(self, data, timestamp):
        """解析系统信息数据"""
        if len(data) >= 3:
            sub_cmd = data[0]
            if sub_cmd == 0x04:  # 电池电压
                voltage_raw = struct.unpack('<H', data[1:3])[0]
                voltage_v = voltage_raw / 1000.0  # 转换为伏特
                self.data_store.system_data['battery_voltage'] = voltage_v
                self.data_store.system_data['last_update'] = timestamp
                
    def _parse_key_event(self, data, timestamp):
        """解析按键事件"""
        if len(data) >= 2:
            key_id = data[0]
            event = data[1]
            
            # 事件类型映射
            event_names = {
                0x01: "按下", 0x02: "长按", 0x04: "长按重复", 0x08: "长按松开",
                0x10: "短按松开", 0x20: "单击", 0x40: "双击", 0x80: "三连击"
            }
            
            self.data_store.key_data.update({
                'key_id': key_id,
                'event': event,
                'event_name': event_names.get(event, f"未知({event:02X})"),
                'last_update': timestamp
            })
            
    def _parse_encoder_data(self, data, timestamp):
        """解析编码器数据"""            
        if len(data) == 37:  # 批量格式: 子命令(1) + 4个电机数据(4×9)
            sub_cmd = data[0]
            if sub_cmd == 0x10:  # 编码器批量上报
                for motor_idx in range(4):
                    offset = 1 + motor_idx * 9
                    motor_data = data[offset:offset+9]
                    
                    if len(motor_data) == 9:
                        motor_id, counter, rps = struct.unpack('<Bif', motor_data)
                        rpm = rps * 60
                        
                        motor_key = f'motor_{motor_idx}'
                        self.data_store.encoder_data[motor_key].update({
                            'id': motor_id,
                            'counter': counter,
                            'rps': rps,
                            'rpm': rpm
                        })
                        
                self.data_store.encoder_data['last_update'] = timestamp
        else:
            print(f"[DEBUG] 编码器数据长度不匹配: 期望37字节，收到{len(data)}字节")
            print(f"[DEBUG] 数据内容: {' '.join(f'{b:02X}' for b in data)}")
                
    def _parse_imu_data(self, data, timestamp):
        """解析IMU数据"""
        if len(data) == 24:  # 6个float: 加速度xyz + 陀螺仪xyz
            values = struct.unpack('<6f', data)
            self.data_store.imu_data.update({
                'accel': {'x': values[0], 'y': values[1], 'z': values[2]},
                'gyro': {'x': values[3], 'y': values[4], 'z': values[5]},
                'last_update': timestamp
            })
        else:
            print(f"[DEBUG] IMU数据长度不匹配: 期望24字节，收到{len(data)}字节")
            
    def _parse_gamepad_data(self, data, timestamp):
        """解析手柄数据"""
        if len(data) == 7:
            buttons, hat, lx, ly, rx, ry = struct.unpack('<HB4b', data)
            self.data_store.gamepad_data.update({
                'buttons': buttons,
                'hat': hat,
                'left_stick': {'x': lx, 'y': ly},
                'right_stick': {'x': rx, 'y': ry},
                'last_update': timestamp
            })
            
    def _parse_sbus_data(self, data, timestamp):
        """解析SBUS数据"""
        if len(data) == 36:
            channels = list(struct.unpack('<16h', data[0:32]))
            ch17, ch18, signal_loss, fail_safe = struct.unpack('<4B', data[32:36])
            
            self.data_store.sbus_data.update({
                'channels': channels,
                'ch17': ch17,
                'ch18': ch18,
                'signal_loss': bool(signal_loss),
                'fail_safe': bool(fail_safe),
                'last_update': timestamp
            })
            
    def _parse_bus_servo_info(self, data, timestamp):
        """解析总线舵机信息"""
        if len(data) == 7:
            self.data_store.bus_servo_data.update({
                'servo_data': list(data),
                'last_update': timestamp
            })

# --- GUI显示器 ---
class RobotDataGUI:
    def __init__(self, data_store):
        self.data_store = data_store
        self.root = tk.Tk()
        self.root.title("STM32机器人控制器数据监控器")
        self.root.geometry("1000x700")
        
        self.setup_gui()
        self.update_timer()
        
    def setup_gui(self):
        # 创建主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建Notebook（标签页）
        notebook = ttk.Notebook(main_frame)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # 系统状态页
        self.create_system_tab(notebook)
        
        # 编码器数据页
        self.create_encoder_tab(notebook)
        
        # IMU数据页
        self.create_imu_tab(notebook)
        
        # 控制输入页
        self.create_input_tab(notebook)
        
        # 统计信息页
        self.create_stats_tab(notebook)
        
    def create_system_tab(self, notebook):
        """创建系统状态标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="系统状态")
        
        # 系统信息
        sys_frame = ttk.LabelFrame(frame, text="系统信息", padding=10)
        sys_frame.pack(fill=tk.X, pady=5)
        
        self.battery_var = tk.StringVar(value="0.00V")
        self.sys_update_var = tk.StringVar(value="未更新")
        
        ttk.Label(sys_frame, text="电池电压:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(sys_frame, textvariable=self.battery_var).grid(row=0, column=1, sticky=tk.W, padx=10)
        
        ttk.Label(sys_frame, text="最后更新:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(sys_frame, textvariable=self.sys_update_var).grid(row=1, column=1, sticky=tk.W, padx=10)
        
        # 按键事件
        key_frame = ttk.LabelFrame(frame, text="按键事件", padding=10)
        key_frame.pack(fill=tk.X, pady=5)
        
        self.key_id_var = tk.StringVar(value="0")
        self.key_event_var = tk.StringVar(value="无事件")
        self.key_update_var = tk.StringVar(value="未更新")
        
        ttk.Label(key_frame, text="按键ID:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(key_frame, textvariable=self.key_id_var).grid(row=0, column=1, sticky=tk.W, padx=10)
        
        ttk.Label(key_frame, text="事件类型:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(key_frame, textvariable=self.key_event_var).grid(row=1, column=1, sticky=tk.W, padx=10)
        
        ttk.Label(key_frame, text="最后更新:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(key_frame, textvariable=self.key_update_var).grid(row=2, column=1, sticky=tk.W, padx=10)
        
    def create_encoder_tab(self, notebook):
        """创建编码器数据标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="编码器数据")
        
        # 创建表格
        columns = ('电机ID', '脉冲计数', '转速(RPS)', '转速(RPM)')
        self.encoder_tree = ttk.Treeview(frame, columns=columns, show='headings', height=8)
        
        for col in columns:
            self.encoder_tree.heading(col, text=col)
            self.encoder_tree.column(col, width=150, anchor=tk.CENTER)
            
        self.encoder_tree.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # 初始化行
        for i in range(4):
            self.encoder_tree.insert('', tk.END, iid=f'motor_{i}', 
                                   values=(f'电机{i}', '0', '0.0000', '0.00'))
        
        # 最后更新时间
        self.encoder_update_var = tk.StringVar(value="未更新")
        ttk.Label(frame, text="最后更新: ").pack(side=tk.LEFT)
        ttk.Label(frame, textvariable=self.encoder_update_var).pack(side=tk.LEFT)
        
    def create_imu_tab(self, notebook):
        """创建IMU数据标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="IMU数据")
        
        # 加速度计
        accel_frame = ttk.LabelFrame(frame, text="加速度计 (m/s²)", padding=10)
        accel_frame.pack(fill=tk.X, pady=5)
        
        self.accel_vars = {
            'x': tk.StringVar(value="0.000"),
            'y': tk.StringVar(value="0.000"),
            'z': tk.StringVar(value="0.000")
        }
        
        for i, axis in enumerate(['x', 'y', 'z']):
            ttk.Label(accel_frame, text=f"{axis.upper()}轴:").grid(row=0, column=i*2, sticky=tk.W, padx=5)
            ttk.Label(accel_frame, textvariable=self.accel_vars[axis]).grid(row=0, column=i*2+1, sticky=tk.W, padx=10)
        
        # 陀螺仪
        gyro_frame = ttk.LabelFrame(frame, text="陀螺仪 (rad/s)", padding=10)
        gyro_frame.pack(fill=tk.X, pady=5)
        
        self.gyro_vars = {
            'x': tk.StringVar(value="0.000"),
            'y': tk.StringVar(value="0.000"),
            'z': tk.StringVar(value="0.000")
        }
        
        for i, axis in enumerate(['x', 'y', 'z']):
            ttk.Label(gyro_frame, text=f"{axis.upper()}轴:").grid(row=0, column=i*2, sticky=tk.W, padx=5)
            ttk.Label(gyro_frame, textvariable=self.gyro_vars[axis]).grid(row=0, column=i*2+1, sticky=tk.W, padx=10)
        
        # 最后更新时间
        self.imu_update_var = tk.StringVar(value="未更新")
        ttk.Label(frame, text="最后更新: ").pack()
        ttk.Label(frame, textvariable=self.imu_update_var).pack()
        
    def create_input_tab(self, notebook):
        """创建控制输入标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="控制输入")
        
        # 手柄数据
        gamepad_frame = ttk.LabelFrame(frame, text="手柄数据", padding=10)
        gamepad_frame.pack(fill=tk.X, pady=5)
        
        self.gamepad_vars = {
            'buttons': tk.StringVar(value="0x0000"),
            'hat': tk.StringVar(value="0"),
            'left_x': tk.StringVar(value="0"),
            'left_y': tk.StringVar(value="0"),
            'right_x': tk.StringVar(value="0"),
            'right_y': tk.StringVar(value="0")
        }
        
        ttk.Label(gamepad_frame, text="按键状态:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(gamepad_frame, textvariable=self.gamepad_vars['buttons']).grid(row=0, column=1, sticky=tk.W, padx=10)
        
        ttk.Label(gamepad_frame, text="方向键:").grid(row=0, column=2, sticky=tk.W, padx=20)
        ttk.Label(gamepad_frame, textvariable=self.gamepad_vars['hat']).grid(row=0, column=3, sticky=tk.W, padx=10)
        
        ttk.Label(gamepad_frame, text="左摇杆:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(gamepad_frame, textvariable=self.gamepad_vars['left_x']).grid(row=1, column=1, sticky=tk.W, padx=10)
        ttk.Label(gamepad_frame, textvariable=self.gamepad_vars['left_y']).grid(row=1, column=2, sticky=tk.W, padx=10)
        
        ttk.Label(gamepad_frame, text="右摇杆:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(gamepad_frame, textvariable=self.gamepad_vars['right_x']).grid(row=2, column=1, sticky=tk.W, padx=10)
        ttk.Label(gamepad_frame, textvariable=self.gamepad_vars['right_y']).grid(row=2, column=2, sticky=tk.W, padx=10)
        
        # SBUS遥控器（简化显示）
        sbus_frame = ttk.LabelFrame(frame, text="SBUS遥控器", padding=10)
        sbus_frame.pack(fill=tk.X, pady=5)
        
        self.sbus_vars = {
            'ch1': tk.StringVar(value="0"),
            'ch2': tk.StringVar(value="0"),
            'ch3': tk.StringVar(value="0"),
            'ch4': tk.StringVar(value="0"),
            'signal_loss': tk.StringVar(value="正常"),
            'fail_safe': tk.StringVar(value="正常")
        }
        
        for i in range(4):
            ttk.Label(sbus_frame, text=f"通道{i+1}:").grid(row=i//2, column=(i%2)*2, sticky=tk.W, padx=5)
            ttk.Label(sbus_frame, textvariable=self.sbus_vars[f'ch{i+1}']).grid(row=i//2, column=(i%2)*2+1, sticky=tk.W, padx=10)
        
        ttk.Label(sbus_frame, text="信号状态:").grid(row=2, column=0, sticky=tk.W, padx=5)
        ttk.Label(sbus_frame, textvariable=self.sbus_vars['signal_loss']).grid(row=2, column=1, sticky=tk.W, padx=10)
        
        ttk.Label(sbus_frame, text="失控保护:").grid(row=2, column=2, sticky=tk.W, padx=5)
        ttk.Label(sbus_frame, textvariable=self.sbus_vars['fail_safe']).grid(row=2, column=3, sticky=tk.W, padx=10)
        
    def create_stats_tab(self, notebook):
        """创建统计信息标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text="统计信息")
        
        stats_frame = ttk.LabelFrame(frame, text="通信统计", padding=10)
        stats_frame.pack(fill=tk.X, pady=5)
        
        self.stats_vars = {
            'total_packets': tk.StringVar(value="0"),
            'valid_packets': tk.StringVar(value="0"),
            'crc_errors': tk.StringVar(value="0"),
            'success_rate': tk.StringVar(value="0.0%"),
            'runtime': tk.StringVar(value="00:00:00")
        }
        
        ttk.Label(stats_frame, text="总接收包数:").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(stats_frame, textvariable=self.stats_vars['total_packets']).grid(row=0, column=1, sticky=tk.W, padx=10)
        
        ttk.Label(stats_frame, text="有效包数:").grid(row=1, column=0, sticky=tk.W)
        ttk.Label(stats_frame, textvariable=self.stats_vars['valid_packets']).grid(row=1, column=1, sticky=tk.W, padx=10)
        
        ttk.Label(stats_frame, text="校验错误:").grid(row=2, column=0, sticky=tk.W)
        ttk.Label(stats_frame, textvariable=self.stats_vars['crc_errors']).grid(row=2, column=1, sticky=tk.W, padx=10)
        
        ttk.Label(stats_frame, text="成功率:").grid(row=3, column=0, sticky=tk.W)
        ttk.Label(stats_frame, textvariable=self.stats_vars['success_rate']).grid(row=3, column=1, sticky=tk.W, padx=10)
        
        ttk.Label(stats_frame, text="运行时间:").grid(row=4, column=0, sticky=tk.W)
        ttk.Label(stats_frame, textvariable=self.stats_vars['runtime']).grid(row=4, column=1, sticky=tk.W, padx=10)
        
        # 数据包计数
        counts_frame = ttk.LabelFrame(frame, text="数据包类型统计", padding=10)
        counts_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.counts_text = scrolledtext.ScrolledText(counts_frame, height=10, width=50)
        self.counts_text.pack(fill=tk.BOTH, expand=True)
        
    def update_display(self):
        """更新GUI显示"""
        try:
            # 更新系统信息
            self.battery_var.set(f"{self.data_store.system_data['battery_voltage']:.2f}V")
            if self.data_store.system_data['last_update']:
                self.sys_update_var.set(datetime.fromtimestamp(self.data_store.system_data['last_update']).strftime("%H:%M:%S"))
            
            # 更新按键事件
            self.key_id_var.set(str(self.data_store.key_data['key_id']))
            self.key_event_var.set(self.data_store.key_data['event_name'])
            if self.data_store.key_data['last_update']:
                self.key_update_var.set(datetime.fromtimestamp(self.data_store.key_data['last_update']).strftime("%H:%M:%S"))
            
            # 更新编码器数据
            for i in range(4):
                motor_key = f'motor_{i}'
                motor_data = self.data_store.encoder_data[motor_key]
                self.encoder_tree.item(motor_key, values=(
                    f"电机{motor_data['id']}",
                    f"{motor_data['counter']:,}",
                    f"{motor_data['rps']:.4f}",
                    f"{motor_data['rpm']:.2f}"
                ))
            
            if self.data_store.encoder_data['last_update']:
                self.encoder_update_var.set(datetime.fromtimestamp(self.data_store.encoder_data['last_update']).strftime("%H:%M:%S"))
            
            # 更新IMU数据
            for axis in ['x', 'y', 'z']:
                self.accel_vars[axis].set(f"{self.data_store.imu_data['accel'][axis]:.3f}")
                self.gyro_vars[axis].set(f"{self.data_store.imu_data['gyro'][axis]:.3f}")
            
            if self.data_store.imu_data['last_update']:
                self.imu_update_var.set(datetime.fromtimestamp(self.data_store.imu_data['last_update']).strftime("%H:%M:%S"))
            
            # 更新手柄数据
            self.gamepad_vars['buttons'].set(f"0x{self.data_store.gamepad_data['buttons']:04X}")
            self.gamepad_vars['hat'].set(str(self.data_store.gamepad_data['hat']))
            self.gamepad_vars['left_x'].set(str(self.data_store.gamepad_data['left_stick']['x']))
            self.gamepad_vars['left_y'].set(str(self.data_store.gamepad_data['left_stick']['y']))
            self.gamepad_vars['right_x'].set(str(self.data_store.gamepad_data['right_stick']['x']))
            self.gamepad_vars['right_y'].set(str(self.data_store.gamepad_data['right_stick']['y']))
            
            # 更新SBUS数据
            for i in range(4):
                self.sbus_vars[f'ch{i+1}'].set(str(self.data_store.sbus_data['channels'][i]))
            
            self.sbus_vars['signal_loss'].set("信号丢失" if self.data_store.sbus_data['signal_loss'] else "正常")
            self.sbus_vars['fail_safe'].set("失控保护" if self.data_store.sbus_data['fail_safe'] else "正常")
            
            # 更新统计信息
            stats = self.data_store.stats
            self.stats_vars['total_packets'].set(str(stats['total_packets']))
            self.stats_vars['valid_packets'].set(str(stats['valid_packets']))
            self.stats_vars['crc_errors'].set(str(stats['crc_errors']))
            
            if stats['total_packets'] > 0:
                success_rate = stats['valid_packets'] / stats['total_packets'] * 100
                self.stats_vars['success_rate'].set(f"{success_rate:.1f}%")
            
            runtime = int(time.time() - stats['start_time'])
            hours, remainder = divmod(runtime, 3600)
            minutes, seconds = divmod(remainder, 60)
            self.stats_vars['runtime'].set(f"{hours:02d}:{minutes:02d}:{seconds:02d}")
            
            # 更新数据包计数
            self.counts_text.delete(1.0, tk.END)
            func_names = {
                0x00: "系统信息", 0x06: "按键事件", 0x07: "编码器数据", 0x08: "总线舵机",
                0x09: "IMU数据", 0x0A: "手柄数据", 0x0B: "SBUS数据", 0x0C: "OLED控制"
            }
            
            for func_code, count in stats['packet_counts'].items():
                name = func_names.get(func_code, f"未知(0x{func_code:02X})")
                self.counts_text.insert(tk.END, f"{name}: {count}\n")
                
        except Exception as e:
            print(f"GUI更新错误: {e}")
    
    def update_timer(self):
        """定时更新GUI"""
        self.update_display()
        self.root.after(100, self.update_timer)  # 每100ms更新一次
        
    def run(self):
        """运行GUI"""
        self.root.mainloop()

# --- 终端显示器 ---
class TerminalDisplay:
    def __init__(self, data_store):
        self.data_store = data_store
        self.last_display_time = 0
        
    def display(self):
        """终端显示数据"""
        current_time = time.time()
        if current_time - self.last_display_time < 1.0:  # 每秒更新一次
            return
            
        self.last_display_time = current_time
        
        # 清屏
        os.system('cls' if os.name == 'nt' else 'clear')
        
        print("=" * 80)
        print("STM32机器人控制器数据监控器")
        print("=" * 80)
        
        # 系统信息
        print(f"电池电压: {self.data_store.system_data['battery_voltage']:.2f}V")
        
        # 编码器数据
        print("\n编码器数据:")
        print("电机ID | 脉冲计数    | 转速(RPS) | 转速(RPM)")
        print("-" * 50)
        for i in range(4):
            motor_data = self.data_store.encoder_data[f'motor_{i}']
            print(f"电机{motor_data['id']}  | {motor_data['counter']:10,} | {motor_data['rps']:8.4f} | {motor_data['rpm']:8.2f}")
        
        # IMU数据
        imu = self.data_store.imu_data
        print(f"\nIMU数据:")
        print(f"加速度: X={imu['accel']['x']:7.3f} Y={imu['accel']['y']:7.3f} Z={imu['accel']['z']:7.3f} m/s²")
        print(f"陀螺仪: X={imu['gyro']['x']:7.3f} Y={imu['gyro']['y']:7.3f} Z={imu['gyro']['z']:7.3f} rad/s")
        
        # 统计信息
        stats = self.data_store.stats
        runtime = int(time.time() - stats['start_time'])
        hours, remainder = divmod(runtime, 3600)
        minutes, seconds = divmod(remainder, 60)
        
        print(f"\n统计信息:")
        print(f"运行时间: {hours:02d}:{minutes:02d}:{seconds:02d}")
        print(f"总包数: {stats['total_packets']} | 有效包: {stats['valid_packets']} | 错误包: {stats['crc_errors']}")
        
        if stats['total_packets'] > 0:
            success_rate = stats['valid_packets'] / stats['total_packets'] * 100
            print(f"成功率: {success_rate:.1f}%")

# --- 主监控类 ---
class RobotDataMonitor:
    def __init__(self, com_port='COM21', baud_rate=1000000, use_gui=True):
        self.com_port = com_port
        self.baud_rate = baud_rate
        self.use_gui = use_gui and GUI_AVAILABLE
        
        self.data_store = RobotDataStore()
        self.parser = RrcProtocolParser()
        self.handler = DataPacketHandler(self.data_store)
        
        self.serial_conn = None
        self.running = False
        self.read_thread = None
        
        if self.use_gui:
            self.gui = RobotDataGUI(self.data_store)
        else:
            self.terminal_display = TerminalDisplay(self.data_store)
        
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.com_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            print(f"✅ 成功连接到 {self.com_port} (波特率: {self.baud_rate})")
            return True
        except serial.SerialException as e:
            print(f"❌ 串口连接失败: {e}")
            return False
    
    def disconnect_serial(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("🔌 串口已断开")
    
    def read_data_loop(self):
        """数据读取循环"""
        print("🔄 开始监听数据...")
        
        while self.running:
            try:
                byte_data = self.serial_conn.read(1)
                if byte_data:
                    packet = self.parser.parse_byte(byte_data[0])
                    if packet:
                        self.handler.handle_packet(packet)
                        
                        # 如果使用终端显示，只在接收到完整数据包时更新
                        if not self.use_gui:
                            self.terminal_display.display()
                    
            except serial.SerialException as e:
                print(f"❌ 串口读取错误: {e}")
                break
            except Exception as e:
                print(f"❌ 数据处理错误: {e}")
                continue
    
    def start_monitoring(self):
        """开始监控"""
        if not self.connect_serial():
            return False
            
        self.running = True
        self.read_thread = threading.Thread(target=self.read_data_loop, daemon=True)
        self.read_thread.start()
        
        return True
    
    def stop_monitoring(self):
        """停止监控"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2.0)
        self.disconnect_serial()
        print("🛑 监控已停止")
    
    def run(self):
        """运行监控器"""
        try:
            if not self.start_monitoring():
                return
            
            if self.use_gui:
                print("🖥️  启动GUI界面...")
                self.gui.run()
            else:
                print("⌨️  使用终端显示模式... (按Ctrl+C停止)")
                try:
                    while True:
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    print("\n用户中断")
                    
        except KeyboardInterrupt:
            print("\n用户中断")
        except Exception as e:
            print(f"❌ 运行错误: {e}")
        finally:
            self.stop_monitoring()

# --- 主函数 ---
def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='STM32机器人控制器数据监控器')
    parser.add_argument('--port', default='COM21', help='串口号 (默认: COM21)')
    parser.add_argument('--baud', type=int, default=1000000, help='波特率 (默认: 1000000)')
    parser.add_argument('--terminal', action='store_true', help='使用终端显示模式')
    
    args = parser.parse_args()
    
    use_gui = not args.terminal
    
    print("🤖 STM32机器人控制器数据监控器")
    print("=" * 50)
    print(f"串口: {args.port}")
    print(f"波特率: {args.baud}")
    print(f"显示模式: {'GUI' if use_gui and GUI_AVAILABLE else '终端'}")
    print("=" * 50)
    
    monitor = RobotDataMonitor(
        com_port=args.port,
        baud_rate=args.baud,
        use_gui=use_gui
    )
    
    monitor.run()

if __name__ == "__main__":
    main()
