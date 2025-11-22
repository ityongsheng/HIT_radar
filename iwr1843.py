import serial
import struct
import numpy as np
from datetime import datetime
import threading
import signal
import time

# 配置参数（根据实际修改）
CLI_PORT = 'COM17'      # 雷达指令端口
DATA_PORT = 'COM18'     # 雷达数据端口
CFG_PATH = r"D:\pythonclass\radar\con_figs.cfg"  # 配置文件路径
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

class TI1843Radar:
    def __init__(self):
        self.cli = serial.Serial(CLI_PORT, baudrate=115200)
        self.data = serial.Serial(DATA_PORT, baudrate=921600)
        self.running = True
        self._send_config()

    def _send_config(self):
        """发送雷达配置文件"""
        with open(CFG_PATH) as f:
            for line in [l.strip() for l in f if l.strip()]:
                self.cli.write(f"{line}\n".encode())
                print(f"[CONFIG] Sent: {line[:40]}...")  # 截断长配置显示
                time.sleep(0.01)

    def _parse_pointcloud(self, buffer):
        try:
            idx = buffer.index(MAGIC_WORD)
            
            # 解析帧头
            header = buffer[idx:idx+40]
            (version, length, platform, 
             frame_num, cpu_cycles, 
             num_obj, num_tlvs) = struct.unpack('<7I', header[8:36])  # 跳过8字节魔数
            
            # 解析TLV头
            tlv_type, tlv_length = struct.unpack('<2I', buffer[idx+40:idx+48])
            
            # 动态计算点数
            num_points = tlv_length // 16  # 每个点16字节
            if num_points > 1000:  # 防止异常数据
                return np.empty((0,4))
            
            # 提取点云数据
            points = []
            data_start = idx + 48  # Header(40) + TLV头(8)
            for offset in range(0, num_points*16, 16):
                point_data = buffer[data_start+offset : data_start+offset+16]
                if len(point_data) < 16: break
                
                x, y, z, vel = struct.unpack('4f', point_data)
                if (x, y, z) != (0, 0, 0):  # 过滤无效点（关键修改点4）
                    points.append([x, y, z, vel])
                    
            return np.array(points)
        except Exception as e:
            print(f"解析失败: {str(e)}")
            return np.empty((0,4))
        
    def start_capture(self, callback):
        """启动数据采集线程"""
        def _capture_loop():
            buffer = b''
            while self.running:
                buffer += self.data.read(self.data.in_waiting)
                if MAGIC_WORD in buffer:
                    points = self._parse_pointcloud(buffer)
                    if len(points) > 0:
                        callback(points)
                    buffer = b''  # 清空已处理数据

        threading.Thread(target=_capture_loop, daemon=True).start()

# 点云打印处理器
def print_points(points):
    print(f"\n[{datetime.now().strftime('%H:%M:%S.%f')}] 收到{len(points)}个点：")
    for i, (x, y, z, vel) in enumerate(points):
        print(f"点{i+1}: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m, 速度={vel:.2f}m/s")


if __name__ == "__main__":
    # 初始化雷达
    radar = TI1843Radar()
    
    # 注册CTRL+C退出
    def signal_handler(sig, frame):
        print("\n正在关闭雷达...")
        radar.running = False
        radar.cli.close()
        radar.data.close()
        exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    # 启动采集
    print("开始采集点云数据（CTRL+C退出）...")
    radar.start_capture(print_points)
    
    # 保持主线程运行
    while True: 
        time.sleep(1)