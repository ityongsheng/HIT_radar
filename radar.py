# -*- coding: utf-8 -*-
# 基础的好看的qtdesigner生成的界面
import sys
import serial
import struct
import numpy as np
from datetime import datetime
import threading
import time
from PyQt5 import QtCore, QtGui, QtWidgets

# 配置参数
CFG_PATH = r"D:\pythonclass\radar\con_figs.cfg"  # 配置文件路径
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'


class TI1843Radar:
    def __init__(self, cli_port=None, data_port=None):
        self.cli_port = cli_port
        self.data_port = data_port
        self.cli = None
        self.data = None
        self.running = False
        self.capturing = False
        self.buffer = b''
        self.points_count = 0
        self.range_profile_length = 0

    def connect(self, cli_port, data_port):
        """连接雷达设备"""
        try:
            self.cli_port = cli_port
            self.data_port = data_port
            self.cli = serial.Serial(cli_port, baudrate=115200, timeout=1)
            self.data = serial.Serial(data_port, baudrate=921600, timeout=1)
            return True
        except Exception as e:
            print(f"连接失败: {str(e)}")
            return False

    def disconnect(self):
        """断开雷达连接"""
        self.stop_capture()
        if self.cli and self.cli.is_open:
            self.cli.close()
        if self.data and self.data.is_open:
            self.data.close()
        self.cli = None
        self.data = None

    def is_connected(self):
        """检查是否已连接"""
        return self.cli is not None and self.cli.is_open and self.data is not None and self.data.is_open

    def send_config(self):
        """发送雷达配置文件"""
        if not self.is_connected():
            return False

        try:
            with open(CFG_PATH) as f:
                for line in [l.strip() for l in f if l.strip()]:
                    self.cli.write(f"{line}\n".encode())
                    time.sleep(0.01)
            return True
        except Exception as e:
            print(f"配置发送失败: {str(e)}")
            return False

    def _parse_pointcloud(self, buffer):
        try:
            idx = buffer.index(MAGIC_WORD)

            # 解析帧头
            header = buffer[idx:idx + 40]
            (version, length, platform,
             frame_num, cpu_cycles,
             num_obj, num_tlvs) = struct.unpack('<7I', header[8:36])

            # 解析TLV头
            tlv_type, tlv_length = struct.unpack('<2I', buffer[idx + 40:idx + 48])

            # 动态计算点数
            num_points = tlv_length // 16
            if num_points > 1000:
                return np.empty((0, 4))

            # 提取点云数据
            points = []
            data_start = idx + 48
            for offset in range(0, num_points * 16, 16):
                point_data = buffer[data_start + offset: data_start + offset + 16]
                if len(point_data) < 16: break

                x, y, z, vel = struct.unpack('4f', point_data)
                if (x, y, z) != (0, 0, 0):
                    points.append([x, y, z, vel])

            return np.array(points)
        except Exception as e:
            return np.empty((0, 4))

    def start_capture(self, callback):
        """启动数据采集线程"""
        if not self.is_connected() or self.capturing:
            return False

        self.capturing = True
        self.running = True
        self.buffer = b''
        self.points_count = 0

        def _capture_loop():
            while self.running:
                try:
                    if self.data.in_waiting > 0:
                        self.buffer += self.data.read(self.data.in_waiting)
                        if MAGIC_WORD in self.buffer:
                            points = self._parse_pointcloud(self.buffer)
                            self.points_count = len(points)
                            if len(points) > 0:
                                callback(points)
                            self.buffer = b''
                    time.sleep(0.01)
                except Exception as e:
                    print(f"数据采集错误: {str(e)}")
                    break
            self.capturing = False

        threading.Thread(target=_capture_loop, daemon=True).start()
        return True

    def stop_capture(self):
        """停止数据采集"""
        self.running = False
        self.capturing = False
        self.buffer = b''
        self.points_count = 0


class RadarController:
    def __init__(self):
        self.radar = TI1843Radar()
        self.temperature = 25.0  # 初始温度
        self.temperature_thread = None
        self.monitoring_temp = False

    def update_temperature(self, callback):
        """模拟温度更新"""

        def _temp_loop():
            while self.monitoring_temp:
                # 模拟温度变化（采集时温度上升，暂停时下降）
                if self.radar.capturing:
                    self.temperature += 0.1
                else:
                    self.temperature -= 0.05

                # 限制温度范围
                self.temperature = max(20, min(80, self.temperature))
                callback(self.temperature)
                time.sleep(1)

        if not self.temperature_thread or not self.temperature_thread.is_alive():
            self.monitoring_temp = True
            self.temperature_thread = threading.Thread(target=_temp_loop, daemon=True)
            self.temperature_thread.start()


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1000, 800)
        # 设置渐变背景
        MainWindow.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #1a2980, stop:1 #26d0ce);
            }
        """)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        # 标题
        self.titleLabel = QtWidgets.QLabel(self.centralwidget)
        self.titleLabel.setGeometry(QtCore.QRect(50, 20, 900, 50))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        self.titleLabel.setFont(font)
        self.titleLabel.setStyleSheet("""
            color: white;
            background-color: rgba(0, 0, 0, 100);
            border-radius: 10px;
            padding: 10px;
        """)
        self.titleLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.titleLabel.setObjectName("titleLabel")

        # 雷达连接控制区域
        self.connectionGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.connectionGroup.setGeometry(QtCore.QRect(50, 90, 400, 200))
        self.connectionGroup.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                color: white;
                border: 2px solid rgba(255, 255, 255, 150);
                border-radius: 15px;
                margin-top: 2ex;
                background-color: rgba(0, 0, 0, 80);
            }
            QGroupBox::title {
                subline-offset: -2px;
                padding: 5px;
                background-color: rgba(0, 100, 200, 150);
                border-radius: 10px;
            }
        """)
        self.connectionGroup.setObjectName("connectionGroup")

        self.labelCliPort = QtWidgets.QLabel(self.connectionGroup)
        self.labelCliPort.setGeometry(QtCore.QRect(30, 50, 100, 30))
        self.labelCliPort.setStyleSheet("color: white; font-size: 12px;")
        self.labelCliPort.setObjectName("labelCliPort")

        self.cliPortCombo = QtWidgets.QComboBox(self.connectionGroup)
        self.cliPortCombo.setGeometry(QtCore.QRect(140, 50, 120, 30))
        self.cliPortCombo.setStyleSheet("""
            QComboBox {
                background-color: rgba(255, 255, 255, 200);
                border: 2px solid #4a90e2;
                border-radius: 10px;
                padding: 5px;
                font-size: 12px;
                color: #333;
            }
            QComboBox::drop-down {
                border: none;
                border-radius: 10px;
            }
            QComboBox::down-arrow {
                image: url(:/images/down_arrow.png);
                width: 15px;
                height: 15px;
                margin-right: 10px;
            }
            QComboBox QAbstractItemView {
                background-color: white;
                border: 1px solid #4a90e2;
                selection-background-color: #4a90e2;
                selection-color: white;
            }
        """)
        self.cliPortCombo.setObjectName("cliPortCombo")

        self.labelDataPort = QtWidgets.QLabel(self.connectionGroup)
        self.labelDataPort.setGeometry(QtCore.QRect(30, 100, 100, 30))
        self.labelDataPort.setStyleSheet("color: white; font-size: 12px;")
        self.labelDataPort.setObjectName("labelDataPort")

        self.dataPortCombo = QtWidgets.QComboBox(self.connectionGroup)
        self.dataPortCombo.setGeometry(QtCore.QRect(140, 100, 120, 30))
        self.dataPortCombo.setStyleSheet("""
            QComboBox {
                background-color: rgba(255, 255, 255, 200);
                border: 2px solid #4a90e2;
                border-radius: 10px;
                padding: 5px;
                font-size: 12px;
                color: #333;
            }
            QComboBox::drop-down {
                border: none;
                border-radius: 10px;
            }
            QComboBox::down-arrow {
                image: url(:/images/down_arrow.png);
                width: 15px;
                height: 15px;
                margin-right: 10px;
            }
            QComboBox QAbstractItemView {
                background-color: white;
                border: 1px solid #4a90e2;
                selection-background-color: #4a90e2;
                selection-color: white;
            }
        """)
        self.dataPortCombo.setObjectName("dataPortCombo")

        self.connectButton = QtWidgets.QPushButton(self.connectionGroup)
        self.connectButton.setGeometry(QtCore.QRect(30, 150, 100, 35))
        self.connectButton.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #4facfe, stop:1 #00f2fe);
                border: none;
                border-radius: 15px;
                color: white;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #00f2fe, stop:1 #4facfe);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #00c6ff, stop:1 #0072ff);
            }
        """)
        self.connectButton.setObjectName("connectButton")

        self.disconnectButton = QtWidgets.QPushButton(self.connectionGroup)
        self.disconnectButton.setGeometry(QtCore.QRect(150, 150, 100, 35))
        self.disconnectButton.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ff416c, stop:1 #ff4b2b);
                border: none;
                border-radius: 15px;
                color: white;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ff4b2b, stop:1 #ff416c);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ff0f4d, stop:1 #d10047);
            }
        """)
        self.disconnectButton.setObjectName("disconnectButton")

        self.sendConfigButton = QtWidgets.QPushButton(self.connectionGroup)
        self.sendConfigButton.setGeometry(QtCore.QRect(270, 150, 100, 35))
        self.sendConfigButton.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #11998e, stop:1 #38ef7d);
                border: none;
                border-radius: 15px;
                color: white;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #38ef7d, stop:1 #11998e);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0fd850, stop:1 #00a050);
            }
        """)
        self.sendConfigButton.setObjectName("sendConfigButton")

        # 数据采集控制区域
        self.captureGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.captureGroup.setGeometry(QtCore.QRect(500, 90, 400, 200))
        self.captureGroup.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                color: white;
                border: 2px solid rgba(255, 255, 255, 150);
                border-radius: 15px;
                margin-top: 2ex;
                background-color: rgba(0, 0, 0, 80);
            }
            QGroupBox::title {
                subline-offset: -2px;
                padding: 5px;
                background-color: rgba(0, 100, 200, 150);
                border-radius: 10px;
            }
        """)
        self.captureGroup.setObjectName("captureGroup")

        self.startCaptureButton = QtWidgets.QPushButton(self.captureGroup)
        self.startCaptureButton.setGeometry(QtCore.QRect(50, 80, 120, 50))
        self.startCaptureButton.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #00b09b, stop:1 #96c93d);
                border: none;
                border-radius: 20px;
                color: white;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #96c93d, stop:1 #00b09b);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #009e60, stop:1 #6fcf00);
            }
        """)
        self.startCaptureButton.setObjectName("startCaptureButton")

        self.stopCaptureButton = QtWidgets.QPushButton(self.captureGroup)
        self.stopCaptureButton.setGeometry(QtCore.QRect(230, 80, 120, 50))
        self.stopCaptureButton.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ff512f, stop:1 #dd2476);
                border: none;
                border-radius: 20px;
                color: white;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #dd2476, stop:1 #ff512f);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #c9185d, stop:1 #e02d45);
            }
        """)
        self.stopCaptureButton.setObjectName("stopCaptureButton")

        # 温度过热保护区域
        self.tempGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.tempGroup.setGeometry(QtCore.QRect(50, 310, 400, 180))
        self.tempGroup.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                color: white;
                border: 2px solid rgba(255, 255, 255, 150);
                border-radius: 15px;
                margin-top: 2ex;
                background-color: rgba(0, 0, 0, 80);
            }
            QGroupBox::title {
                subline-offset: -2px;
                padding: 5px;
                background-color: rgba(0, 100, 200, 150);
                border-radius: 10px;
            }
        """)
        self.tempGroup.setObjectName("tempGroup")

        self.tempLabel = QtWidgets.QLabel(self.tempGroup)
        self.tempLabel.setGeometry(QtCore.QRect(30, 60, 120, 30))
        self.tempLabel.setStyleSheet("color: white; font-size: 14px; font-weight: bold;")
        self.tempLabel.setObjectName("tempLabel")

        self.tempValueLabel = QtWidgets.QLabel(self.tempGroup)
        self.tempValueLabel.setGeometry(QtCore.QRect(160, 60, 100, 30))
        self.tempValueLabel.setStyleSheet("""
            color: #3498db;
            font-weight: bold;
            font-size: 16px;
            background-color: rgba(255, 255, 255, 150);
            border-radius: 10px;
            padding: 5px;
        """)
        self.tempValueLabel.setObjectName("tempValueLabel")

        self.tempProtectionButton = QtWidgets.QPushButton(self.tempGroup)
        self.tempProtectionButton.setGeometry(QtCore.QRect(30, 120, 120, 40))
        self.tempProtectionButton.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffd89b, stop:1 #19547b);
                border: none;
                border-radius: 15px;
                color: white;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #19547b, stop:1 #ffd89b);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #144463, stop:1 #e6c38a);
            }
        """)
        self.tempProtectionButton.setObjectName("tempProtectionButton")

        self.resumeButton = QtWidgets.QPushButton(self.tempGroup)
        self.resumeButton.setGeometry(QtCore.QRect(230, 120, 120, 40))
        self.resumeButton.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #7F00FF, stop:1 #E100FF);
                border: none;
                border-radius: 15px;
                color: white;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #E100FF, stop:1 #7F00FF);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #6a00d9, stop:1 #c000d9);
            }
        """)
        self.resumeButton.setObjectName("resumeButton")

        # 状态信息区域
        self.statusGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.statusGroup.setGeometry(QtCore.QRect(500, 310, 400, 180))
        self.statusGroup.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                color: white;
                border: 2px solid rgba(255, 255, 255, 150);
                border-radius: 15px;
                margin-top: 2ex;
                background-color: rgba(0, 0, 0, 80);
            }
            QGroupBox::title {
                subline-offset: -2px;
                padding: 5px;
                background-color: rgba(0, 100, 200, 150);
                border-radius: 10px;
            }
        """)
        self.statusGroup.setObjectName("statusGroup")

        self.statusLabel = QtWidgets.QLabel(self.statusGroup)
        self.statusLabel.setGeometry(QtCore.QRect(30, 50, 80, 30))
        self.statusLabel.setStyleSheet("color: white; font-size: 12px;")
        self.statusLabel.setObjectName("statusLabel")

        self.statusValueLabel = QtWidgets.QLabel(self.statusGroup)
        self.statusValueLabel.setGeometry(QtCore.QRect(120, 50, 250, 30))
        self.statusValueLabel.setStyleSheet("""
            color: #e74c3c;
            font-weight: bold;
            font-size: 12px;
            background-color: rgba(255, 255, 255, 150);
            border-radius: 10px;
            padding: 5px;
        """)
        self.statusValueLabel.setObjectName("statusValueLabel")

        self.pointCountLabel = QtWidgets.QLabel(self.statusGroup)
        self.pointCountLabel.setGeometry(QtCore.QRect(30, 90, 100, 30))
        self.pointCountLabel.setStyleSheet("color: white; font-size: 12px;")
        self.pointCountLabel.setObjectName("pointCountLabel")

        self.pointCountValueLabel = QtWidgets.QLabel(self.statusGroup)
        self.pointCountValueLabel.setGeometry(QtCore.QRect(140, 90, 80, 30))
        self.pointCountValueLabel.setStyleSheet("""
            color: #3498db;
            font-weight: bold;
            font-size: 14px;
            background-color: rgba(255, 255, 255, 150);
            border-radius: 10px;
            padding: 5px;
            text-align: center;
        """)
        self.pointCountValueLabel.setObjectName("pointCountValueLabel")

        self.rangeProfileLabel = QtWidgets.QLabel(self.statusGroup)
        self.rangeProfileLabel.setGeometry(QtCore.QRect(30, 130, 100, 30))
        self.rangeProfileLabel.setStyleSheet("color: white; font-size: 12px;")
        self.rangeProfileLabel.setObjectName("rangeProfileLabel")

        self.rangeProfileValueLabel = QtWidgets.QLabel(self.statusGroup)
        self.rangeProfileValueLabel.setGeometry(QtCore.QRect(140, 130, 80, 30))
        self.rangeProfileValueLabel.setStyleSheet("""
            color: #3498db;
            font-weight: bold;
            font-size: 14px;
            background-color: rgba(255, 255, 255, 150);
            border-radius: 10px;
            padding: 5px;
            text-align: center;
        """)
        self.rangeProfileValueLabel.setObjectName("rangeProfileValueLabel")

        # 系统日志区域
        self.logGroup = QtWidgets.QGroupBox(self.centralwidget)
        self.logGroup.setGeometry(QtCore.QRect(50, 510, 850, 250))
        self.logGroup.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                color: white;
                border: 2px solid rgba(255, 255, 255, 150);
                border-radius: 15px;
                margin-top: 2ex;
                background-color: rgba(0, 0, 0, 80);
            }
            QGroupBox::title {
                subline-offset: -2px;
                padding: 5px;
                background-color: rgba(0, 100, 200, 150);
                border-radius: 10px;
            }
        """)
        self.logGroup.setObjectName("logGroup")

        self.logTextEdit = QtWidgets.QTextEdit(self.logGroup)
        self.logTextEdit.setGeometry(QtCore.QRect(20, 40, 810, 190))
        self.logTextEdit.setStyleSheet("""
            background-color: rgba(255, 255, 255, 220);
            border: 2px solid #4a90e2;
            border-radius: 10px;
            font-family: Consolas, Monaco, monospace;
            font-size: 11px;
            padding: 10px;
        """)
        self.logTextEdit.setReadOnly(True)
        self.logTextEdit.setObjectName("logTextEdit")

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1000, 30))
        self.menubar.setStyleSheet("""
            QMenuBar {
                background-color: rgba(0, 0, 0, 100);
                color: white;
                border-bottom: 1px solid #4a90e2;
            }
            QMenuBar::item {
                background: transparent;
                padding: 8px 15px;
            }
            QMenuBar::item:selected {
                background: rgba(74, 144, 226, 150);
            }
            QMenuBar::item:pressed {
                background: #4a90e2;
            }
        """)
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)

        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setStyleSheet("""
            QStatusBar {
                background-color: rgba(0, 0, 0, 100);
                color: white;
                border-top: 1px solid #4a90e2;
            }
        """)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "TI IWR1843 雷达控制面板 | 酷炫版"))
        self.titleLabel.setText(_translate("MainWindow", "TI IWR1843 毫米波雷达高级控制系统"))
        self.connectionGroup.setTitle(_translate("MainWindow", "📡 雷达连接控制"))
        self.labelCliPort.setText(_translate("MainWindow", "CLI端口："))
        self.labelDataPort.setText(_translate("MainWindow", "数据端口："))
        self.connectButton.setText(_translate("MainWindow", "一键连接"))
        self.disconnectButton.setText(_translate("MainWindow", "断开连接"))
        self.sendConfigButton.setText(_translate("MainWindow", "发送配置"))
        self.captureGroup.setTitle(_translate("MainWindow", "📊 数据采集控制"))
        self.startCaptureButton.setText(_translate("MainWindow", "开始采集"))
        self.stopCaptureButton.setText(_translate("MainWindow", "停止采集"))
        self.tempGroup.setTitle(_translate("MainWindow", "🌡️ 温度过热保护"))
        self.tempLabel.setText(_translate("MainWindow", "当前温度："))
        self.tempValueLabel.setText(_translate("MainWindow", "25.0°C"))
        self.tempProtectionButton.setText(_translate("MainWindow", "雷达暂歇"))
        self.resumeButton.setText(_translate("MainWindow", "恢复工作"))
        self.statusGroup.setTitle(_translate("MainWindow", "📈 状态信息"))
        self.statusLabel.setText(_translate("MainWindow", "状态："))
        self.statusValueLabel.setText(_translate("MainWindow", "未连接，请连接"))
        self.pointCountLabel.setText(_translate("MainWindow", "点云数量："))
        self.pointCountValueLabel.setText(_translate("MainWindow", "0"))
        self.rangeProfileLabel.setText(_translate("MainWindow", "距离像长度："))
        self.rangeProfileValueLabel.setText(_translate("MainWindow", "0"))
        self.logGroup.setTitle(_translate("MainWindow", "📋 系统日志"))


class MainWindow(QtWidgets.QMainWindow):
    # 定义信号
    update_log_signal = QtCore.pyqtSignal(str)
    update_temp_signal = QtCore.pyqtSignal(float)
    update_status_signal = QtCore.pyqtSignal(str)
    update_points_signal = QtCore.pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # 初始化控制器
        self.controller = RadarController()

        # 连接信号与槽
        self.update_log_signal.connect(self.update_log)
        self.update_temp_signal.connect(self.update_temperature)
        self.update_status_signal.connect(self.update_status)
        self.update_points_signal.connect(self.update_points)

        # 连接按钮事件
        self.ui.connectButton.clicked.connect(self.connect_radar)
        self.ui.disconnectButton.clicked.connect(self.disconnect_radar)
        self.ui.sendConfigButton.clicked.connect(self.send_config)
        self.ui.startCaptureButton.clicked.connect(self.start_capture)
        self.ui.stopCaptureButton.clicked.connect(self.stop_capture)
        self.ui.tempProtectionButton.clicked.connect(self.pause_radar)
        self.ui.resumeButton.clicked.connect(self.resume_radar)

        # 填充串口列表
        self.populate_serial_ports()

        # 开始温度监控
        self.controller.update_temperature(self.temp_callback)

        # 初始化状态
        self.update_status("未连接，请连接")

    def populate_serial_ports(self):
        """填充串口列表"""
        ports = ['COM1', 'COM2', 'COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8',
                 'COM9', 'COM10', 'COM11', 'COM12', 'COM13', 'COM14', 'COM15',
                 'COM16', 'COM17', 'COM18', 'COM19', 'COM20']
        self.ui.cliPortCombo.addItems(ports)
        self.ui.dataPortCombo.addItems(ports)

        # 设置默认值
        if 'COM17' in ports:
            self.ui.cliPortCombo.setCurrentText('COM17')
        if 'COM18' in ports:
            self.ui.dataPortCombo.setCurrentText('COM18')

    def log_message(self, message):
        """添加日志信息"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        formatted_message = f"[{timestamp}] {message}"
        self.update_log_signal.emit(formatted_message)

    def update_log(self, message):
        """更新日志显示"""
        self.ui.logTextEdit.append(message)
        self.ui.logTextEdit.moveCursor(QtGui.QTextCursor.End)

    def temp_callback(self, temperature):
        """温度更新回调"""
        self.update_temp_signal.emit(temperature)

    def update_temperature(self, temperature):
        """更新温度显示"""
        self.ui.tempValueLabel.setText(f"{temperature:.1f}°C")

        # 温度过高警告
        if temperature > 70:
            self.ui.tempValueLabel.setStyleSheet("""
                color: white;
                font-weight: bold;
                font-size: 16px;
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ff416c, stop:1 #ff4b2b);
                border-radius: 10px;
                padding: 5px;
            """)
            self.log_message("⚠️ 警告：温度过高，建议暂停雷达工作")
        elif temperature > 60:
            self.ui.tempValueLabel.setStyleSheet("""
                color: white;
                font-weight: bold;
                font-size: 16px;
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffd89b, stop:1 #19547b);
                border-radius: 10px;
                padding: 5px;
            """)
        else:
            self.ui.tempValueLabel.setStyleSheet("""
                color: white;
                font-weight: bold;
                font-size: 16px;
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #00b09b, stop:1 #96c93d);
                border-radius: 10px;
                padding: 5px;
            """)

    def update_status(self, status):
        """更新状态显示"""
        self.ui.statusValueLabel.setText(status)
        if "已连接" in status:
            self.ui.statusValueLabel.setStyleSheet("""
                color: white;
                font-weight: bold;
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #00b09b, stop:1 #96c93d);
                border-radius: 10px;
                padding: 5px;
            """)
        elif "未连接" in status:
            self.ui.statusValueLabel.setStyleSheet("""
                color: white;
                font-weight: bold;
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ff512f, stop:1 #dd2476);
                border-radius: 10px;
                padding: 5px;
            """)
        else:
            self.ui.statusValueLabel.setStyleSheet("""
                color: white;
                font-weight: bold;
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ffd89b, stop:1 #19547b);
                border-radius: 10px;
                padding: 5px;
            """)

    def update_points(self, count):
        """更新点云数量显示"""
        self.ui.pointCountValueLabel.setText(str(count))

    def connect_radar(self):
        """连接雷达"""
        cli_port = self.ui.cliPortCombo.currentText()
        data_port = self.ui.dataPortCombo.currentText()

        if self.controller.radar.connect(cli_port, data_port):
            self.log_message(f"✅ 雷达已连接到 CLI端口: {cli_port}, 数据端口: {data_port}")
            self.update_status("已连接")
        else:
            self.log_message("❌ 雷达连接失败，请检查端口设置")
            self.update_status("连接失败")

    def disconnect_radar(self):
        """断开雷达连接"""
        self.controller.radar.disconnect()
        self.log_message("🔌 雷达连接已断开")
        self.update_status("未连接，请连接")
        self.ui.pointCountValueLabel.setText("0")

    def send_config(self):
        """发送配置"""
        if not self.controller.radar.is_connected():
            self.log_message("❌ 错误：雷达未连接，无法发送配置")
            return

        if self.controller.radar.send_config():
            self.log_message("📤 配置文件已发送到雷达")
        else:
            self.log_message("❌ 配置发送失败")

    def start_capture(self):
        """开始采集"""
        if not self.controller.radar.is_connected():
            self.log_message("❌ 错误：雷达未连接，无法开始采集")
            return

        if self.controller.radar.start_capture(self.capture_callback):
            self.log_message("▶️ 开始采集点云数据")
            self.update_status("正在采集中...")
        else:
            self.log_message("❌ 采集启动失败")

    def capture_callback(self, points):
        """采集数据回调"""
        self.update_points_signal.emit(len(points))

        # 模拟距离像长度更新
        range_profile_length = len(points) * 2
        self.ui.rangeProfileValueLabel.setText(str(range_profile_length))

        # 记录日志（仅记录前几个点）
        if len(points) > 0:
            log_msg = f"📊 收到 {len(points)} 个点云数据"
            if len(points) > 5:
                log_msg += "，前5个点: "
                for i in range(min(5, len(points))):
                    x, y, z, vel = points[i]
                    log_msg += f"({x:.2f}, {y:.2f}, {z:.2f}) "
            else:
                log_msg += "，点: "
                for point in points:
                    x, y, z, vel = point
                    log_msg += f"({x:.2f}, {y:.2f}, {z:.2f}) "
            self.log_message(log_msg)

    def stop_capture(self):
        """停止采集"""
        self.controller.radar.stop_capture()
        self.log_message("⏹️ 已停止采集点云数据")
        self.update_status("已连接")
        self.ui.pointCountValueLabel.setText("0")
        self.ui.rangeProfileValueLabel.setText("0")

    def pause_radar(self):
        """暂停雷达工作（温度保护）"""
        if self.controller.radar.capturing:
            self.controller.radar.stop_capture()
            self.log_message("❄️ 温度过高，雷达已暂停工作")
            self.update_status("已暂停（温度保护）")
        else:
            self.log_message("ℹ️ 雷达当前未在工作")

    def resume_radar(self):
        """恢复雷达工作"""
        if self.controller.radar.is_connected() and not self.controller.radar.capturing:
            self.start_capture()
            self.log_message("▶️ 雷达已恢复工作")
        elif not self.controller.radar.is_connected():
            self.log_message("❌ 错误：雷达未连接，无法恢复工作")
        else:
            self.log_message("ℹ️ 雷达已在工作中")


def main():
    app = QtWidgets.QApplication(sys.argv)
    # 设置应用程序样式
    app.setStyle("Fusion")

    # 创建调色板实现深色主题
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(25, 25, 25))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ToolTipBase, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.ToolTipText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Text, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ButtonText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.BrightText, QtCore.Qt.red)
    palette.setColor(QtGui.QPalette.Link, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.HighlightedText, QtCore.Qt.black)
    app.setPalette(palette)

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
