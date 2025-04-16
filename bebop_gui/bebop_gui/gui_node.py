#!/usr/bin/env python3

import rclpy
import sys
import math
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel,
    QComboBox, QStackedWidget, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import cv2
from cv_bridge import CvBridge
from enum import IntEnum
from threading import Lock

class DroneState(IntEnum):
    IDLE = 0
    AUTOMATIC = 1
    TAKING_OFF = 2
    LANDING = 3
    EMERGENCY_STOP = 4
    RESTART_TRAJ = 6

class BebopGUI(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, 'bebop_gui_combined')
        QMainWindow.__init__(self)

        self.setup_ros()
        self.setup_ui()

        self.state_lock = Lock()
        self.current_state = DroneState.IDLE
        self.goal_positions = []
        self.bebop_positions = []
        self.bridge = CvBridge()

    def setup_ros(self):
        qos = QoSProfile(depth=10)
        self.state_publisher = self.create_publisher(Int32, '/state', qos)
        self.goal_subscription = self.create_subscription(
            Pose, '/goal', self.goal_callback, qos)
        self.pose_subscription = self.create_subscription(
            Pose, '/bebop1/pose', self.bebop1_callback, qos)
        self.image_subscription = self.create_subscription(
            Image, '/world/bebop/model/bebop1/link/body/sensor/rgb_camera_sensor/image', self.image_callback, qos)

    def setup_ui(self):
        self.setWindowTitle('Bebop Drone Controller')
        self.setFixedSize(800, 900)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        self.layout = QVBoxLayout()
        central_widget.setLayout(self.layout)

        # Título
        title = QLabel('Bebop Control Panel')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 20px; font-weight: bold; color: #333;')
        self.layout.addWidget(title)

        # Botones
        self.setup_buttons(self.layout)

        # Combo selector
        self.view_selector = QComboBox()
        self.view_selector.addItems(["Cámara Bebop1", "Gráfica 3D"])
        self.view_selector.currentIndexChanged.connect(self.switch_view)
        self.layout.addWidget(self.view_selector)

        # Widget de contenido (imagen o gráfica)
        self.stack = QStackedWidget()
        self.setup_camera_view()
        self.setup_plot()
        self.layout.addWidget(self.stack)

        # Estado
        self.status_label = QLabel('Status: IDLE')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet('font-size: 16px; color: #555;')
        self.layout.addWidget(self.status_label)

    def setup_buttons(self, layout):
        button_style = """
            QPushButton {
                padding: 12px;
                font-size: 16px;
                border-radius: 6px;
                margin: 8px;
                min-height: 40px;
            }
            QPushButton:hover { background-color: #e0e0e0; }
        """
        buttons = [
            ('Automatic', self.automatic, '#FF9800'),
            ('Take Off', self.takeoff, '#4CAF50'),
            ('Land', self.land, '#f44336'),
            ('EMERGENCY STOP', self.emergency_stop, '#d32f2f'),
            ('Reset Trajectory', self.restart_trajectory, '#9C27B0')
        ]
        for text, callback, color in buttons:
            btn = QPushButton(text)
            btn.setStyleSheet(f"{button_style} QPushButton {{ background-color: {color}; color: white; }}")
            btn.clicked.connect(callback)
            layout.addWidget(btn)

    def setup_camera_view(self):
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.stack.addWidget(self.image_label)

    def setup_plot(self):
        self.figure = Figure(figsize=(6, 5), tight_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_title("3D Trajectory", pad=20)
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Z [m]")
        self.stack.addWidget(self.canvas)

    def switch_view(self, index):
        self.stack.setCurrentIndex(index)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            h, w = cv_image.shape[:2]
            ratio = min(640/w, 480/h)
            cv_image = cv2.resize(cv_image, (int(w*ratio), int(h*ratio)))
            
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            q_img = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            
            self.image_label.setPixmap(QPixmap.fromImage(q_img))
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")

    def update_plot(self):
        self.ax.clear()
        
        if self.goal_positions:
            x_goal, y_goal, z_goal = zip(*self.goal_positions)
            self.ax.plot(x_goal, y_goal, z_goal, 'b-o', linewidth=2, markersize=5, label='Goal')
            self.ax.scatter(x_goal[-1], y_goal[-1], z_goal[-1], c='r', s=100, marker='*', label='Current Goal')
        
        if self.bebop_positions:
            x_bebop, y_bebop, z_bebop = zip(*self.bebop_positions)
            self.ax.plot(x_bebop, y_bebop, z_bebop, 'g--s', linewidth=1, markersize=3, label='Bebop Pose')
            self.ax.scatter(x_bebop[-1], y_bebop[-1], z_bebop[-1], c='m', s=80, marker='D', label='Current Pose')
        
        self.ax.set_title("3D Trajectory", pad=20)
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Z [m]")
        self.ax.legend()
        self.canvas.draw()

    def publish_state(self, state):
        with self.state_lock:
            try:
                msg = Int32()
                msg.data = int(state)
                self.state_publisher.publish(msg)
                self.current_state = state
                self.update_status_label()
                self.get_logger().info(f"Published state: {state.name}")
            except Exception as e:
                self.get_logger().error(f"Error publishing state: {str(e)}")

    def update_status_label(self):
        state_names = {
            DroneState.IDLE: "IDLE",
            DroneState.AUTOMATIC: "AUTOMATIC",
            DroneState.TAKING_OFF: "TAKING OFF",
            DroneState.LANDING: "LANDING",
            DroneState.EMERGENCY_STOP: "EMERGENCY STOP",
            DroneState.RESTART_TRAJ: "RESET TRAJECTORY"
        }
        self.status_label.setText(f"Status: {state_names.get(self.current_state, 'UNKNOWN')}")

    def goal_callback(self, msg):
        try:
            if not all(math.isfinite(getattr(msg.position, axis)) for axis in ['x', 'y', 'z']):
                raise ValueError("Invalid position values")
            pos = msg.position
            self.goal_positions.append((pos.x, pos.y, pos.z))
            self.update_plot()
        except Exception as e:
            self.get_logger().error(f"Error processing goal: {str(e)}")
            self.show_warning("Error", f"Invalid goal position: {str(e)}")

    def bebop1_callback(self, msg):
        try:
            if not all(math.isfinite(getattr(msg.position, axis)) for axis in ['x', 'y', 'z']):
                raise ValueError("Invalid position values")
            pos = msg.position
            self.bebop_positions.append((pos.x, pos.y, pos.z))
            self.update_plot()
        except Exception as e:
            self.get_logger().error(f"Error processing bebop pose: {str(e)}")

    def automatic(self): self.publish_state(DroneState.AUTOMATIC)
    def takeoff(self): self.publish_state(DroneState.TAKING_OFF)
    def land(self): self.publish_state(DroneState.LANDING)
    def emergency_stop(self): self.publish_state(DroneState.EMERGENCY_STOP)
    def restart_trajectory(self):
        self.goal_positions.clear()
        self.bebop_positions.clear()
        self.update_plot()
        self.publish_state(DroneState.RESTART_TRAJ)

    def show_warning(self, title, message):
        QMessageBox.warning(self, title, message)

    def closeEvent(self, event):
        self.cleanup()
        event.accept()
        
    def cleanup(self):
        """Cleanup resources before closing"""
        self.publish_state(DroneState.IDLE)
        self.destroy_node()
        if hasattr(self, 'timer'):
            self.timer.stop()
        QApplication.quit()

def sigint_handler(*args):
    """Handler for the SIGINT signal (Ctrl+C)"""
    QApplication.quit()

def main(args=None):
    rclpy.init(args=args)
    
    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, sigint_handler)
    
    app = QApplication([])
    gui = BebopGUI()
    gui.show()

    # Configurar timer para procesar eventos ROS
    gui.timer = QTimer()
    gui.timer.timeout.connect(lambda: rclpy.spin_once(gui, timeout_sec=0.01))
    gui.timer.start(10)

    # Configurar para cerrar todo cuando la aplicación termine
    app.aboutToQuit.connect(gui.cleanup)

    # Ejecutar aplicación con manejo de Ctrl+C
    timer = QTimer()
    timer.start(500)  # Intervalo para verificar Ctrl+C
    timer.timeout.connect(lambda: None)  # Mantener la aplicación responsiva

    # Ejecutar aplicación
    ret = app.exec_()
    
    # Limpieza final por si acaso
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()