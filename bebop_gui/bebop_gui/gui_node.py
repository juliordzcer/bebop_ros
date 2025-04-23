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
    QComboBox, QStackedWidget, QMessageBox, QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import cv2
from cv_bridge import CvBridge
from enum import IntEnum
from threading import Lock
import numpy as np

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
        self.error_history = {'x': [], 'y': [], 'z': [], 'yaw': []}
        self.time_history = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.current_goal = None
        self.current_pose = None
        self.bridge = CvBridge()

    def setup_ros(self):
        qos = QoSProfile(depth=10)
        self.state_publisher = self.create_publisher(Int32, '/state', qos)
        self.goal_subscription = self.create_subscription(
            Pose, '/goal', self.goal_callback, qos)
        self.pose_subscription = self.create_subscription(
            Pose, '/bebop1/pose', self.bebop1_callback, qos)
        self.image_subscription = self.create_subscription(
            Image, '/world/bebop/model/bebop1/link/body/sensor/rgb_camera_sensor/image', 
            self.image_callback, qos)

    def setup_ui(self):
        self.setWindowTitle('Bebop Drone Controller')
        self.setFixedSize(1200, 1000)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        self.main_layout = QVBoxLayout()
        central_widget.setLayout(self.main_layout)

        # Título
        title = QLabel('Bebop Control Panel')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 20px; font-weight: bold; color: #333;')
        self.main_layout.addWidget(title)

        # Botones
        self.setup_buttons()

        # Combo selector
        self.view_selector = QComboBox()
        self.view_selector.addItems(["Cámara Bebop1", "Gráfica 3D", "Errores de Control"])
        self.view_selector.currentIndexChanged.connect(self.switch_view)
        self.main_layout.addWidget(self.view_selector)

        # Widget de contenido
        self.stack = QStackedWidget()
        self.setup_camera_view()
        self.setup_3d_plot()
        self.setup_error_plot()
        self.main_layout.addWidget(self.stack)

        # Estado
        self.status_label = QLabel('Status: IDLE')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet('font-size: 16px; color: #555;')
        self.main_layout.addWidget(self.status_label)

    def setup_buttons(self):
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
        
        button_layout = QHBoxLayout()
        for text, callback, color in buttons:
            btn = QPushButton(text)
            btn.setStyleSheet(f"{button_style} QPushButton {{ background-color: {color}; color: white; }}")
            btn.clicked.connect(callback)
            button_layout.addWidget(btn)
        
        self.main_layout.addLayout(button_layout)

    def setup_camera_view(self):
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.stack.addWidget(self.image_label)

    def setup_3d_plot(self):
        self.figure_3d = Figure(figsize=(10, 8), tight_layout=True)
        self.canvas_3d = FigureCanvas(self.figure_3d)
        self.ax_3d = self.figure_3d.add_subplot(111, projection='3d')
        self.ax_3d.set_title("3D Trajectory", pad=20)
        self.ax_3d.set_xlabel("X [m]")
        self.ax_3d.set_ylabel("Y [m]")
        self.ax_3d.set_zlabel("Z [m]")
        self.stack.addWidget(self.canvas_3d)

    def setup_error_plot(self):
        self.figure_error = Figure(figsize=(10, 8), tight_layout=True)
        self.canvas_error = FigureCanvas(self.figure_error)
        
        # Crear 4 subplots
        self.ax_x = self.figure_error.add_subplot(221)
        self.ax_y = self.figure_error.add_subplot(222)
        self.ax_z = self.figure_error.add_subplot(223)
        self.ax_yaw = self.figure_error.add_subplot(224)
        
        # Configurar ejes
        self.ax_x.set_title("Error en X")
        self.ax_x.set_ylabel("Error [m]")
        self.ax_x.grid(True)
        
        self.ax_y.set_title("Error en Y")
        self.ax_y.set_ylabel("Error [m]")
        self.ax_y.grid(True)
        
        self.ax_z.set_title("Error en Z")
        self.ax_z.set_xlabel("Tiempo [s]")
        self.ax_z.set_ylabel("Error [m]")
        self.ax_z.grid(True)
        
        self.ax_yaw.set_title("Error en Yaw")
        self.ax_yaw.set_xlabel("Tiempo [s]")
        self.ax_yaw.set_ylabel("Error [rad]")
        self.ax_yaw.grid(True)
        
        self.stack.addWidget(self.canvas_error)

    def switch_view(self, index):
        self.stack.setCurrentIndex(index)
        if index == 1:  # Gráfica 3D
            self.update_3d_plot()
        elif index == 2:  # Gráfica de errores
            self.update_error_plot()

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
            self.get_logger().error(f"Error en image_callback: {str(e)}")

    def update_3d_plot(self):
        self.ax_3d.clear()
        
        if self.goal_positions:
            x_goal, y_goal, z_goal = zip(*self.goal_positions)
            self.ax_3d.plot(x_goal, y_goal, z_goal, 'b-o', linewidth=2, markersize=5, label='Goal')
            if self.current_goal:
                self.ax_3d.scatter(self.current_goal[0], self.current_goal[1], self.current_goal[2], 
                                  c='r', s=100, marker='*', label='Current Goal')
        
        if self.bebop_positions:
            x_bebop, y_bebop, z_bebop = zip(*self.bebop_positions)
            self.ax_3d.plot(x_bebop, y_bebop, z_bebop, 'g--s', linewidth=1, markersize=3, label='Bebop Pose')
            if self.current_pose:
                self.ax_3d.scatter(self.current_pose[0], self.current_pose[1], self.current_pose[2], 
                                  c='m', s=80, marker='D', label='Current Pose')
        
        self.ax_3d.set_title("3D Trajectory", pad=20)
        self.ax_3d.set_xlabel("X [m]")
        self.ax_3d.set_ylabel("Y [m]")
        self.ax_3d.set_zlabel("Z [m]")
        self.ax_3d.legend()
        self.canvas_3d.draw()

    def update_error_plot(self):
        if not self.time_history:
            return
            
        # Actualizar gráfica de error en X
        self.ax_x.clear()
        self.ax_x.plot(self.time_history, self.error_history['x'], 'r-', label='Error X')
        self.ax_x.set_title("Error en X")
        self.ax_x.set_ylabel("Error [m]")
        self.ax_x.grid(True)
        self.ax_x.legend()
        
        # Actualizar gráfica de error en Y
        self.ax_y.clear()
        self.ax_y.plot(self.time_history, self.error_history['y'], 'g-', label='Error Y')
        self.ax_y.set_title("Error en Y")
        self.ax_y.set_ylabel("Error [m]")
        self.ax_y.grid(True)
        self.ax_y.legend()
        
        # Actualizar gráfica de error en Z
        self.ax_z.clear()
        self.ax_z.plot(self.time_history, self.error_history['z'], 'b-', label='Error Z')
        self.ax_z.set_title("Error en Z")
        self.ax_z.set_xlabel("Tiempo [s]")
        self.ax_z.set_ylabel("Error [m]")
        self.ax_z.grid(True)
        self.ax_z.legend()
        
        # Actualizar gráfica de error en Yaw
        self.ax_yaw.clear()
        self.ax_yaw.plot(self.time_history, self.error_history['yaw'], 'm-', label='Error Yaw')
        self.ax_yaw.set_title("Error en Yaw")
        self.ax_yaw.set_xlabel("Tiempo [s]")
        self.ax_yaw.set_ylabel("Error [rad]")
        self.ax_yaw.grid(True)
        self.ax_yaw.legend()
        
        self.canvas_error.draw()

    def calculate_errors(self):
        if self.current_goal is None or self.current_pose is None:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        
        # Calcular errores
        error_x = self.current_goal[0] - self.current_pose[0]
        error_y = self.current_goal[1] - self.current_pose[1]
        error_z = self.current_goal[2] - self.current_pose[2]
        
        # Para yaw necesitamos calcular la diferencia angular correcta
        error_yaw = self.current_goal[3] - self.current_pose[3]
        error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi  # Normalizar a [-π, π]
        
        # Almacenar errores
        self.error_history['x'].append(error_x)
        self.error_history['y'].append(error_y)
        self.error_history['z'].append(error_z)
        self.error_history['yaw'].append(error_yaw)
        self.time_history.append(current_time)
        
        # Limitar el historial
        max_history = 1000
        if len(self.time_history) > max_history:
            self.time_history = self.time_history[-max_history:]
            for key in self.error_history:
                self.error_history[key] = self.error_history[key][-max_history:]
        
        # Actualizar gráfica si está visible
        if self.stack.currentIndex() == 2:
            self.update_error_plot()

    def goal_callback(self, msg):
        try:
            if not all(math.isfinite(getattr(msg.position, axis)) for axis in ['x', 'y', 'z']):
                raise ValueError("Invalid position values")
            
            pos = msg.position
            orientation = msg.orientation
            self.current_goal = (
                pos.x, 
                pos.y, 
                pos.z,
                self.quaternion_to_yaw(orientation) if orientation else 0.0
            )
            
            self.goal_positions.append((pos.x, pos.y, pos.z))
            if self.stack.currentIndex() == 1:
                self.update_3d_plot()
                
            self.calculate_errors()
            
        except Exception as e:
            self.get_logger().error(f"Error processing goal: {str(e)}")
            self.show_warning("Error", f"Invalid goal position: {str(e)}")

    def bebop1_callback(self, msg):
        try:
            if not all(math.isfinite(getattr(msg.position, axis)) for axis in ['x', 'y', 'z']):
                raise ValueError("Invalid position values")
            
            pos = msg.position
            orientation = msg.orientation
            self.current_pose = (
                pos.x, 
                pos.y, 
                pos.z,
                self.quaternion_to_yaw(orientation) if orientation else 0.0
            )
            
            self.bebop_positions.append((pos.x, pos.y, pos.z))
            if self.stack.currentIndex() == 1:
                self.update_3d_plot()
                
            self.calculate_errors()
            
        except Exception as e:
            self.get_logger().error(f"Error processing bebop pose: {str(e)}")

    def quaternion_to_yaw(self, q):
        # Convertir cuaternión a ángulo de yaw (en radianes)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # Resto de métodos permanecen igual...
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

    def automatic(self): self.publish_state(DroneState.AUTOMATIC)
    def takeoff(self): self.publish_state(DroneState.TAKING_OFF)
    def land(self): self.publish_state(DroneState.LANDING)
    def emergency_stop(self): self.publish_state(DroneState.EMERGENCY_STOP)
    def restart_trajectory(self):
        self.goal_positions.clear()
        self.bebop_positions.clear()
        self.error_history = {'x': [], 'y': [], 'z': [], 'yaw': []}
        self.time_history = []
        if self.stack.currentIndex() == 1:
            self.update_3d_plot()
        elif self.stack.currentIndex() == 2:
            self.update_error_plot()
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