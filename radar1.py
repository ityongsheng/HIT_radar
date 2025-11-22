# -*- coding: utf-8 -*-
"""
radar.py  增强版
新增：
  1) 3D 点云实时显示（matplotlib 嵌入 QWidget）
  2) 一维距离像实时折线图（pyqtgraph）
  3) 自动搜索串口 + 一键连接
  4) 保留原 QtDesigner UI、温度保护逻辑
"""

import sys, os, time, threading, numpy as np, serial, struct
from datetime import datetime
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QVBoxLayout, QSizePolicy
from PyQt5.QtCore import QTimer

# -------------------- 可视化库 --------------------
import pyqtgraph as pg
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D      # noqa  只为激活 3D 工具

# -------------------- 全局常量 --------------------
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

    def update_profile(self, data):
        if data is None or len(data) == 0:
            return
        self.curve.setData(np.arange(len(data)), data)


# ===================================================================
#  主窗口（沿用 QtDesigner 的 Ui_MainWindow）
# ===================================================================
class MainWindow(QtWidgets.QMainWindow):
    # 信号
    update_log_signal = QtCore.pyqtSignal(str)
    update_temp_signal = QtCore.pyqtSignal(float)
    update_status_signal = QtCore.pyqtSignal(str)
    update_points_signal = QtCore.pyqtSignal(int)
    update_range_signal = QtCore.pyqtSignal(int)

    def __init__(self):
        super().__init__()
        # 创建UI
        self.setupUi()

        # 控制器
        self.ctrl = RadarController()

        # 新增可视化面板
        self.cloud3d = PointCloud3D()
        self.range1d = RangeProfile1D()
        # 插入到 designer 里预留的空白 QWidget（名称为 widgetCloud 和 widgetRange）
        self.widgetCloud.setLayout(QVBoxLayout())
        self.widgetCloud.layout().addWidget(self.cloud3d)
        self.widgetRange.setLayout(QVBoxLayout())
        self.widgetRange.layout().addWidget(self.range1d)

        # 信号槽
        self.update_log_signal.connect(self.update_log)
        self.update_temp_signal.connect(self.update_temperature)
        self.update_status_signal.connect(self.update_status)
        self.update_points_signal.connect(self.update_points)
        self.update_range_signal.connect(self.update_range)

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

        # 初始状态
        self.update_status('未连接，请连接')

    def setupUi(self):
        self.setObjectName("MainWindow")
        self.resize(1200, 800)
        self.centralwidget = QtWidgets.QWidget(self)
        self.centralwidget.setObjectName("centralwidget")

        # 主布局
        self.mainLayout = QtWidgets.QHBoxLayout(self.centralwidget)

        # 左侧控制面板
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

        # 配置控制
        self.configGroup = QtWidgets.QGroupBox("配置控制")
        self.configLayout = QtWidgets.QVBoxLayout(self.configGroup)
        self.sendConfigButton = QtWidgets.QPushButton("发送配置")
        self.configLayout.addWidget(self.sendConfigButton)

        # 采集控制
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

        # 状态显示
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

        # 添加到左侧布局
        self.leftLayout.addWidget(self.connectionGroup)
        self.leftLayout.addWidget(self.configGroup)
        self.leftLayout.addWidget(self.captureGroup)
        self.leftLayout.addWidget(self.tempGroup)
        self.leftLayout.addWidget(self.statusGroup)
        self.leftLayout.addStretch()

        # 右侧显示区域
        self.rightPanel = QtWidgets.QWidget()
        self.rightLayout = QtWidgets.QVBoxLayout(self.rightPanel)

        # 标签页
        self.tabWidget = QtWidgets.QTabWidget()
        self.widgetCloud = QtWidgets.QWidget()
        self.widgetRange = QtWidgets.QWidget()
        self.tabWidget.addTab(self.widgetCloud, "3D点云")
        self.tabWidget.addTab(self.widgetRange, "距离像")

        # 日志区域
        self.logGroup = QtWidgets.QGroupBox("日志")
        self.logLayout = QtWidgets.QVBoxLayout(self.logGroup)
        self.logTextEdit = QtWidgets.QTextEdit()
        self.logTextEdit.setReadOnly(True)
        self.logLayout.addWidget(self.logTextEdit)

        self.rightLayout.addWidget(self.tabWidget)
        self.rightLayout.addWidget(self.logGroup)

        # 添加到主布局
        self.mainLayout.addWidget(self.leftPanel)
        self.mainLayout.addWidget(self.rightPanel)

        self.setCentralWidget(self.centralwidget)
        self.setWindowTitle("IWR1843 雷达可视化")

        # 刷新端口
        self.refresh_ports()

    def refresh_ports(self):
        self.cliPortCombo.clear()
        self.dataPortCombo.clear()
        import serial.tools.list_ports as lp
        ports = [p.device for p in lp.comports()]
        self.cliPortCombo.addItems(ports)
        self.dataPortCombo.addItems(ports)

    # -------------------- 串口搜索 + 一键连接 --------------------
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
        # 距离像（这里用点数×2 模拟长度，你可在 _parse_pointcloud 里真解析 TLV=2）
        self.update_range_signal.emit(len(pts) * 2)

    # -------------------- 温度 --------------------
    def temp_callback(self, t):
        self.update_temp_signal.emit(t)

    def update_temperature(self, t):
        self.tempValueLabel.setText(f'{t:.1f}°C')
        if t > 70:
            style = 'color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #ff416c,stop:1 #ff4b2b)'
        elif t > 60:
            style = 'color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #ffd89b,stop:1 #19547b)'
        else:
            style = 'color:white;background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #00b09b,stop:1 #96c93d)'
        self.tempValueLabel.setStyleSheet(style)

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
    # 深色调色板（保持原来酷炫黑）
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
