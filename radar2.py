# -*- coding: utf-8 -*-
"""
radar.py  增强版
新增：
  1) 3D 点云实时显示（matplotlib 嵌入 QWidget）
  2) 一维距离像实时折线图（pyqtgraph）
  3) 自动搜索串口 + 一键连接
  4) 保留原 QtDesigner UI、温度保护逻辑
  5) ===== 新增：DirectionDisplay 水平目标方位指示器 =====
"""

import sys, os, time, threading, numpy as np, serial, struct
from datetime import datetime
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QVBoxLayout, QSizePolicy
from PyQt5.QtCore import QTimer
import math
from PyQt5 import QtWidgets, QtCore, QtGui

# -------------------- 可视化库 --------------------
import pyqtgraph as pg
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D      # noqa  只为激活 3D 工具

# -------------------- 全局常量 --------------------
from radar1 import RangeProfile1D

CFG_PATH = r"D:\pythonclass\radar\con_figs.cfg"
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'
# 串口关键字（自动识别用）
CLI_KEYWORDS = ['enhanced', 'application', 'uart (enhanced)']
DATA_KEYWORDS = ['standard', 'auxiliary', 'uart (standard)']

# ===================================================================
#  雷达底层通信类（保持原接口不变）
# ===================================================================
class TI1843Radar:
    def __init__(self, cli_port=None, data_port=None):
        self.cli_port = cli_port
        self.data_port = data_port
        self.cli = None
        self.data = None
        self.running = False
        self.capturing = False
        self.buffer = b''

    # ------------ 连接/断开 ------------
    def connect(self, cli_port, data_port):
        try:
            self.cli = serial.Serial(cli_port, 115200, timeout=1)
            self.data = serial.Serial(data_port, 921600, timeout=1)
            return True
        except Exception as e:
            print(f'连接失败: {e}')
            return False

    def disconnect(self):
        self.stop_capture()
        for s in (self.cli, self.data):
            if s and s.is_open:
                s.close()
        self.cli = self.data = None

    def is_connected(self):
        return self.cli and self.cli.is_open and self.data and self.data.is_open

    # ------------ 发送配置 ------------
    def send_config(self):
        if not self.is_connected():
            return False
        try:
            with open(CFG_PATH) as f:
                for line in [l.strip() for l in f if l.strip()]:
                    self.cli.write(f'{line}\n'.encode())
                    time.sleep(0.01)
            return True
        except Exception as e:
            print(f'配置发送失败: {e}')
            return False

    # ------------ 数据解析 ------------
    def _parse_pointcloud(self, buffer):
        try:
            idx = buffer.index(MAGIC_WORD)
            header = buffer[idx:idx + 40]
            version, length, platform, frame_num, cpu_cycles, num_obj, num_tlvs = \
                struct.unpack('<7I', header[8:36])
            tlv_type, tlv_length = struct.unpack('<2I', buffer[idx + 40:idx + 48])
            if tlv_type != 1:
                return np.empty((0, 4))
            num_points = tlv_length // 16
            if num_points > 1000:
                return np.empty((0, 4))
            points = []
            off = idx + 48
            for i in range(num_points):
                x, y, z, vel = struct.unpack('4f', buffer[off + i * 16:off + i * 16 + 16])
                if (x, y, z) != (0, 0, 0):
                    points.append([x, y, z, vel])
            return np.array(points)
        except Exception:
            return np.empty((0, 4))

    # ------------ 采集线程 ------------
    def start_capture(self, callback):
        if self.capturing or not self.is_connected():
            return False
        self.running = self.capturing = True
        self.buffer = b''

        def loop():
            while self.running:
                try:
                    if self.data.in_waiting:
                        self.buffer += self.data.read(self.data.in_waiting)
                        if MAGIC_WORD in self.buffer:
                            pts = self._parse_pointcloud(self.buffer)
                            if len(pts):
                                callback(pts)
                            self.buffer = b''
                    time.sleep(0.01)
                except Exception as e:
                    print(f'采集异常: {e}')
                    break
            self.capturing = False

        threading.Thread(target=loop, daemon=True).start()
        return True

    def stop_capture(self):
        self.running = False
        self.capturing = False


# ===================================================================
#  温度模拟（保持原逻辑）
# ===================================================================
class RadarController:
    def __init__(self):
        self.radar = TI1843Radar()
        self.temperature = 25.0
        self.monitoring = False
        self.t_thread = None

    def update_temperature(self, callback):
        self.monitoring = True

        def loop():
            while self.monitoring:
                if self.radar.capturing:
                    self.temperature += 0.1
                else:
                    self.temperature -= 0.05
                self.temperature = max(20, min(80, self.temperature))
                callback(self.temperature)
                time.sleep(1)

        if not self.t_thread or not self.t_thread.is_alive():
            self.t_thread = threading.Thread(target=loop, daemon=True)
            self.t_thread.start()


# ===================================================================
#  自动串口搜索
# ===================================================================
def search_radar_ports():
    import serial.tools.list_ports as lp
    cli_port = data_port = None
    ports = list(lp.comports())

    for p in ports:
        desc = f"{p.description} {p.hwid}".lower()
        if not cli_port and any(k in desc for k in CLI_KEYWORDS):
            cli_port = p.device
        if not data_port and any(k in desc for k in DATA_KEYWORDS):
            data_port = p.device

    # 兜底策略
    if not cli_port or not data_port:
        sorted_p = sorted([p.device for p in ports])
        if len(sorted_p) >= 2:
            cli_port = cli_port or sorted_p[1]
            data_port = data_port or sorted_p[0]
        elif len(sorted_p) == 1:
            cli_port = cli_port or sorted_p[0]
    return cli_port, data_port


# ===================================================================
#  3D 点云窗口（可嵌入 QWidget）
# ===================================================================
class PointCloud3D(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.fig = Figure(facecolor='black')
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('black')
        self.scatter = self.ax.scatter([], [], [], s=15, alpha=0.7)
        self.ax.set_xlabel('X (m)', color='white')
        self.ax.set_ylabel('Y (m)', color='white')
        self.ax.set_zlabel('Z (m)', color='white')
        self.ax.tick_params(colors='white')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.canvas)

    def update_points(self, pts):
        if pts is None or len(pts) == 0:
            return
        self.ax.clear()
        self.ax.set_xlabel('X (m)', color='white')
        self.ax.set_ylabel('Y (m)', color='white')
        self.ax.set_zlabel('Z (m)', color='white')
        self.ax.tick_params(colors='white')
        self.ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                        c=pts[:, 3], s=15, alpha=0.7, cmap='jet')
        self.canvas.draw()


# ===================================================================
#  一维距离像窗口
# ===================================================================
class RangeProfile1D(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.plot = pg.PlotWidget()
            self.plot.setBackground('k')
            self.plot.showGrid(x=True, y=True, alpha=0.3)
            self.plot.setLabel('left', 'Amplitude', 'dB')
            self.plot.setLabel('bottom', 'Range', 'm')
            self.curve = self.plot.plot(pen='y')
            lay = QVBoxLayout(self)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.addWidget(self.plot)

        def update_profile(self, data: bytes):
            """
            更新距离像曲线
            :param data: bytes 格式的距离像数据
            """
            if not data:
                return

            try:
                # 每 4 字节一个 float32
                num_bins = len(data) // 4
                range_profile = np.frombuffer(data, dtype='<f4', count=num_bins)

                # 如果想画真实距离，把 x 轴换成  range_bin * dr
                # dr = 0.3 m  # 举例：距离分辨率
                # x = np.arange(num_bins) * dr
                x = np.arange(num_bins)
                self.curve.setData(x, range_profile)
            except Exception as e:
                print("距离像解析失败:", e)
                self.curve.clear()

# ===================================================================
#  方向指示器（DirectionDisplay 已并入）
# ===================================================================
class DirectionDisplay(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.target_angle = 0
        self.setMinimumSize(360, 180)

    def set_target_angle(self, angle):
        self.target_angle = angle % 360
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(240, 240, 240))
        w, h = self.width(), self.height()
        cy = h // 2
        # 主刻度 30°
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 2))
        font = painter.font(); font.setPointSize(8); painter.setFont(font)
        for ang in range(0, 361, 30):
            x = int(w * ang / 360)
            painter.drawLine(x, cy - 10, x, cy + 10)
            text = f"{ang}°"
            tw = painter.boundingRect(0, 0, 50, 20, QtCore.Qt.AlignCenter, text).width()
            painter.drawText(x - tw // 2, cy + 25, text)
        # 副刻度 10°
        painter.setPen(QtGui.QPen(QtGui.QColor(100, 100, 100), 1))
        for ang in range(0, 360, 10):
            if ang % 30 != 0:
                x = int(w * ang / 360)
                painter.drawLine(x, cy - 5, x, cy + 5)
        # 基线
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 255), 2))
        painter.drawLine(0, cy, w, cy)
        # 箭头
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 0, 0), 3))
        painter.setBrush(QtGui.QBrush(QtGui.QColor(255, 0, 0)))
        ax = int(w * self.target_angle / 360)
        head = QtGui.QPolygon([QtCore.QPoint(ax, cy - 15),
                               QtCore.QPoint(ax + 8, cy),
                               QtCore.QPoint(ax - 8, cy)])
        painter.drawPolygon(head)


# ===================================================================
#  主窗口
# ===================================================================
class MainWindow(QtWidgets.QMainWindow):
    # -------------- 信号 --------------
    update_log_signal   = QtCore.pyqtSignal(str)
    update_temp_signal  = QtCore.pyqtSignal(float)
    update_status_signal= QtCore.pyqtSignal(str)
    update_points_signal= QtCore.pyqtSignal(int)
    update_range_signal = QtCore.pyqtSignal(int)
    #### 新增 ####
    update_angle_signal = QtCore.pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.setupUi()
        self.ctrl = RadarController()

        # 可视化面板
        self.cloud3d = PointCloud3D()
        self.range1d = RangeProfile1D()
        #### 新增 ####
        self.dirWidget = DirectionDisplay()
        self.dirWidget.setFixedHeight(200)

        self.widgetCloud.setLayout(QVBoxLayout())
        self.widgetCloud.layout().addWidget(self.cloud3d)
        self.widgetRange.setLayout(QVBoxLayout())
        self.widgetRange.layout().addWidget(self.range1d)
        #### 新增 ####
        self.leftLayout.addWidget(self.dirWidget)

        # 信号槽
        self.update_log_signal.connect(self.update_log)
        self.update_temp_signal.connect(self.update_temperature)
        self.update_status_signal.connect(self.update_status)
        self.update_points_signal.connect(self.update_points)
        self.update_range_signal.connect(self.update_range)
        #### 新增 ####
        self.update_angle_signal.connect(self.update_angle)

        # 按钮
        self.connectButton.clicked.connect(self.auto_connect)
        self.disconnectButton.clicked.connect(self.disconnect_radar)
        self.sendConfigButton.clicked.connect(self.send_cfg)
        self.startCaptureButton.clicked.connect(self.start_cap)
        self.stopCaptureButton.clicked.connect(self.stop_cap)
        self.tempProtectionButton.clicked.connect(self.pause_radar)
        self.resumeButton.clicked.connect(self.resume_radar)

        # 温度线程
        self.ctrl.update_temperature(self.temp_callback)
        self.update_status('未连接，请连接')

    # -------------------- UI 生成 --------------------
    def setupUi(self):
        self.setObjectName("MainWindow")
        self.resize(1200, 800)
        self.centralwidget = QtWidgets.QWidget(self)
        self.mainLayout = QtWidgets.QHBoxLayout(self.centralwidget)

        # 左侧
        self.leftPanel = QtWidgets.QWidget()
        self.leftPanel.setFixedWidth(300)
        self.leftLayout = QtWidgets.QVBoxLayout(self.leftPanel)

        # 连接控制
        self.connectionGroup = QtWidgets.QGroupBox("连接控制")
        self.connectionLayout = QtWidgets.QVBoxLayout(self.connectionGroup)
        self.portLayout = QtWidgets.QFormLayout()
        self.cliPortCombo = QtWidgets.QComboBox()
        self.dataPortCombo = QtWidgets.QComboBox()
        self.refreshPortsButton = QtWidgets.QPushButton("刷新端口")
        self.refreshPortsButton.clicked.connect(self.refresh_ports)
        self.portLayout.addRow("CLI端口:", self.cliPortCombo)
        self.portLayout.addRow("数据端口:", self.dataPortCombo)
        self.portLayout.addRow("", self.refreshPortsButton)
        self.buttonLayout = QtWidgets.QHBoxLayout()
        self.connectButton = QtWidgets.QPushButton("自动连接")
        self.disconnectButton = QtWidgets.QPushButton("断开连接")
        self.buttonLayout.addWidget(self.connectButton)
        self.buttonLayout.addWidget(self.disconnectButton)
        self.connectionLayout.addLayout(self.portLayout)
        self.connectionLayout.addLayout(self.buttonLayout)

        # 配置
        self.configGroup = QtWidgets.QGroupBox("配置控制")
        self.configLayout = QtWidgets.QVBoxLayout(self.configGroup)
        self.sendConfigButton = QtWidgets.QPushButton("发送配置")
        self.configLayout.addWidget(self.sendConfigButton)

        # 采集
        self.captureGroup = QtWidgets.QGroupBox("采集控制")
        self.captureLayout = QtWidgets.QVBoxLayout(self.captureGroup)
        self.startCaptureButton = QtWidgets.QPushButton("开始采集")
        self.stopCaptureButton = QtWidgets.QPushButton("停止采集")
        self.captureLayout.addWidget(self.startCaptureButton)
        self.captureLayout.addWidget(self.stopCaptureButton)

        # 温度保护
        self.tempGroup = QtWidgets.QGroupBox("温度保护")
        self.tempLayout = QtWidgets.QVBoxLayout(self.tempGroup)
        self.tempProtectionButton = QtWidgets.QPushButton("暂停采集")
        self.resumeButton = QtWidgets.QPushButton("恢复采集")
        self.tempLayout.addWidget(self.tempProtectionButton)
        self.tempLayout.addWidget(self.resumeButton)

        # 状态
        self.statusGroup = QtWidgets.QGroupBox("状态")
        self.statusLayout = QtWidgets.QFormLayout(self.statusGroup)
        self.statusValueLabel = QtWidgets.QLabel("未连接")
        self.tempValueLabel = QtWidgets.QLabel("25.0°C")
        self.pointCountValueLabel = QtWidgets.QLabel("0")
        self.rangeProfileValueLabel = QtWidgets.QLabel("0")
        self.statusLayout.addRow("状态:", self.statusValueLabel)
        self.statusLayout.addRow("温度:", self.tempValueLabel)
        self.statusLayout.addRow("点数:", self.pointCountValueLabel)
        self.statusLayout.addRow("距离像长度:", self.rangeProfileValueLabel)

        self.leftLayout.addWidget(self.connectionGroup)
        self.leftLayout.addWidget(self.configGroup)
        self.leftLayout.addWidget(self.captureGroup)
        self.leftLayout.addWidget(self.tempGroup)
        self.leftLayout.addWidget(self.statusGroup)
        self.leftLayout.addStretch()

        # 右侧
        self.rightPanel = QtWidgets.QWidget()
        self.rightLayout = QtWidgets.QVBoxLayout(self.rightPanel)
        self.tabWidget = QtWidgets.QTabWidget()
        self.widgetCloud = QtWidgets.QWidget()
        self.widgetRange = QtWidgets.QWidget()
        self.tabWidget.addTab(self.widgetCloud, "3D点云")
        self.tabWidget.addTab(self.widgetRange, "距离像")
        self.logGroup = QtWidgets.QGroupBox("日志")
        self.logLayout = QtWidgets.QVBoxLayout(self.logGroup)
        self.logTextEdit = QtWidgets.QTextEdit()
        self.logTextEdit.setReadOnly(True)
        self.logLayout.addWidget(self.logTextEdit)
        self.rightLayout.addWidget(self.tabWidget)
        self.rightLayout.addWidget(self.logGroup)

        self.mainLayout.addWidget(self.leftPanel)
        self.mainLayout.addWidget(self.rightPanel)
        self.setCentralWidget(self.centralwidget)
        self.setWindowTitle("IWR1843 雷达可视化")
        self.refresh_ports()

    # -------------------- 端口刷新 --------------------
    def refresh_ports(self):
        self.cliPortCombo.clear()
        self.dataPortCombo.clear()
        import serial.tools.list_ports as lp
        ports = [p.device for p in lp.comports()]
        self.cliPortCombo.addItems(ports)
        self.dataPortCombo.addItems(ports)

    # -------------------- 自动连接 --------------------
    def auto_connect(self):
        cli, data = search_radar_ports()
        if not cli or not data:
            self.log_message('❌ 未识别到雷达串口，请手动选择')
            return
        self.cliPortCombo.setCurrentText(cli)
        self.dataPortCombo.setCurrentText(data)
        self.connect_radar()

    def connect_radar(self):
        cli = self.cliPortCombo.currentText()
        data = self.dataPortCombo.currentText()
        if self.ctrl.radar.connect(cli, data):
            self.log_message(f'✅ 已连接  CLI={cli}  DATA={data}')
            self.update_status('已连接')
        else:
            self.log_message('❌ 连接失败')
            self.update_status('连接失败')

    def disconnect_radar(self):
        self.ctrl.radar.disconnect()
        self.log_message('🔌 已断开')
        self.update_status('未连接')

    # -------------------- 配置 --------------------
    def send_cfg(self):
        if self.ctrl.radar.send_config():
            self.log_message('📤 配置已发送')
        else:
            self.log_message('❌ 配置发送失败')

    # -------------------- 采集 --------------------
    def start_cap(self):
        if not self.ctrl.radar.is_connected():
            self.log_message('❌ 请先连接雷达')
            return
        if self.ctrl.radar.start_capture(self.data_callback):
            self.log_message('▶️ 开始采集')
            self.update_status('采集中...')

    def stop_cap(self):
        self.ctrl.radar.stop_capture()
        self.log_message('⏹️ 停止采集')
        self.update_status('已连接')

    # -------------------- 数据回调 --------------------
    def data_callback(self, pts):
        self.ctrl.radar.capturing = True
        # 3D 点云
        self.cloud3d.update_points(pts)
        # 点计数
        self.update_points_signal.emit(len(pts))
        # 距离像（模拟）
        if len(pts) > 0:
            range_data = self.generate_range_profile_from_points(pts)
            self.range1d.update_profile(range_data)

        #### 新增：计算水平方位角并发射 ####
        if len(pts):
            idx = np.argmax(pts[:, 3])      # 选速度最强点
            x, y = pts[idx, 0], pts[idx, 1]
            angle = math.degrees(math.atan2(y, x)) % 360
            self.update_angle_signal.emit(angle)

    def generate_range_profile_from_points(self, points):
        """
        从点云数据生成距离像
        :param points: numpy array of shape (n, 4) [x, y, z, velocity]
        :return: bytes 格式的距离像数据
        """
        # 计算每个点的距离
        distances = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2 + points[:, 2] ** 2)

        # 设置距离分辨率和最大距离
        distance_resolution = 0.1  # m
        max_distance = 10.0  # m

        # 创建距离bin数组
        num_bins = int(max_distance / distance_resolution)
        range_profile = np.zeros(num_bins)

        # 统计每个距离bin的点数
        for d in distances:
            if d < max_distance:
                bin_idx = int(d / distance_resolution)
                range_profile[bin_idx] += 1
        # 转换为float32字节数据
        return range_profile.astype(np.float32).tobytes()
    # -------------------- 温度 --------------------
    def temp_callback(self, t):
        self.update_temp_signal.emit(t)

    def update_temperature(self, t):
        try:
            # 原有温度更新逻辑
            self.tempValueLabel.setText(f'{t:.1f}°C')
            if t > 70:
                style = 'color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #ff416c,stop:1 #ff4b2b)'
            elif t > 60:
                style = 'color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #ffd89b,stop:1 #19547b)'
            else:
                style = 'color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #00b09b,stop:1 #96c93d)'
            self.tempValueLabel.setStyleSheet(style)
        except KeyboardInterrupt:
            print("温度更新被用户中断")
            # 可以在这里添加清理逻辑
            raise  # 重新抛出异常让程序正常退出

    # -------------------- 状态 / 日志 --------------------
    def update_status(self, txt):
        self.statusValueLabel.setText(txt)

    def update_points(self, n):
        self.pointCountValueLabel.setText(str(n))

    def update_range(self, n):
        self.rangeProfileValueLabel.setText(str(n))

    def log_message(self, msg):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.logTextEdit.append(f'[{timestamp}] {msg}')

    def update_log(self, msg):
        self.logTextEdit.append(msg)
        self.logTextEdit.moveCursor(QtGui.QTextCursor.End)

    #### 新增 ####
    def update_angle(self, angle):
        self.dirWidget.set_target_angle(angle)

    # -------------------- 温度保护 --------------------
    def pause_radar(self):
        if self.ctrl.radar.capturing:
            self.stop_cap()
            self.log_message('❄️ 温度过高，已暂停')

    def resume_radar(self):
        if self.ctrl.radar.is_connected() and not self.ctrl.radar.capturing:
            self.start_cap()
            self.log_message('▶️ 已恢复工作')


# ===================================================================
#  启动
# ===================================================================
def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(25, 25, 25))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.Text, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ButtonText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.HighlightedText, QtCore.Qt.black)
    app.setPalette(palette)

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())



if __name__ == '__main__':
    main()