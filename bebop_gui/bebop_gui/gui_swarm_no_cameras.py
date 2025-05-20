#!/usr/bin/env python3

import rclpy
import sys
import math
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

import time

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel,
    QComboBox, QStackedWidget, QMessageBox, QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

import matplotlib
matplotlib.use('Qt5Agg')
matplotlib.rcParams['path.simplify'] = True
matplotlib.rcParams['path.simplify_threshold'] = 1.0

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

        self.declare_parameter('num_drones', 20)
        num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value

        self.robot_names = [f"bebop{i+1}" for i in range(num_drones)]
    
        self.setup_ros()
        self.setup_ui()

        self.state_lock = Lock()
        self.current_state = DroneState.IDLE
        self.goal_positions = []
        self.drone_positions = {name: [] for name in self.robot_names}
        self.error_history = {'formation': []}
        self.time_history = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.current_goal = None
        self.current_poses = {name: None for name in self.robot_names}
        self.last_update_time = 0

    def setup_ros(self):
        qos = QoSProfile(depth=10)
        self.state_publisher = self.create_publisher(Int32, '/state', qos)
        
        self.pose_subs = []
        for name in self.robot_names:
            self.pose_subs.append(
                self.create_subscription(
                    Pose, f'/{name}/pose',
                    lambda msg, n=name: self.pose_callback(msg, n),
                    qos
                )
            )
        
        self.formation_error_sub = self.create_subscription(
            Float32MultiArray, '/formation_errors',
            self.formation_error_callback, qos
        )

    def setup_ui(self):
        self.setWindowTitle('Bebop Drone Controller')
        self.setFixedSize(1400, 800)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        self.main_layout = QVBoxLayout()
        central_widget.setLayout(self.main_layout)

        title = QLabel(f'Bebop Control Panel - {len(self.robot_names)} Drones')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 20px; font-weight: bold; color: #333;')
        self.main_layout.addWidget(title)

        self.setup_buttons()
        self.setup_selectors()

        self.stack = QStackedWidget()
        self.setup_3d_plot()
        self.setup_error_plot()
        self.main_layout.addWidget(self.stack)

        self.status_label = QLabel('Status: IDLE')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet('font-size: 16px; color: #555;')
        self.main_layout.addWidget(self.status_label)

        self.view_update_timer = QTimer()
        self.view_update_timer.timeout.connect(self.update_current_view)
        self.view_update_timer.start(100)

    def setup_selectors(self):
        self.view_selector = QComboBox()
        self.view_selector.addItems(["Gráfica 3D", "Error de Formación"])
        self.view_selector.currentIndexChanged.connect(self.switch_view)
        self.main_layout.addWidget(self.view_selector)

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
        self.ax_error = self.figure_error.add_subplot(111)
        self.ax_error.set_title("Error de Formación")
        self.ax_error.set_xlabel("Tiempo [s]")
        self.ax_error.set_ylabel("Error [m]")
        self.ax_error.grid(True)
        self.stack.addWidget(self.canvas_error)

    def switch_view(self, index):
        self.stack.setCurrentIndex(index)
        if index == 0:  # Gráfica 3D
            self.update_3d_plot()
        elif index == 1:  # Gráfica de error
            self.update_error_plot()

    def pose_callback(self, msg, drone_name):
        try:
            if not all(math.isfinite(getattr(msg.position, axis)) for axis in ['x', 'y', 'z']):
                raise ValueError("Invalid position values")
            
            pos = msg.position
            orientation = msg.orientation
            self.current_poses[drone_name] = (
                pos.x, 
                pos.y, 
                pos.z,
                self.quaternion_to_yaw(orientation) if orientation else 0.0
            )
            
            self.drone_positions[drone_name].append((pos.x, pos.y, pos.z))
            if self.stack.currentIndex() == 0:
                self.update_3d_plot()
                
        except Exception as e:
            self.get_logger().error(f"Error processing {drone_name} pose: {str(e)}")

    def formation_error_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        
        if len(msg.data) == len(self.robot_names):
            avg_error = sum(msg.data) / len(msg.data)
            self.error_history['formation'].append(avg_error)
            self.time_history.append(current_time)
            
            max_history = 1000
            if len(self.time_history) > max_history:
                self.time_history = self.time_history[-max_history:]
                self.error_history['formation'] = self.error_history['formation'][-max_history:]
            
            if self.stack.currentIndex() == 1:
                self.update_error_plot()

    def update_3d_plot(self):
        try:
            self.ax_3d.cla()
            
            if not hasattr(self, 'plot_initialized'):
                self.ax_3d.set_title(f"3D Trajectory - {len(self.robot_names)} Drones")
                self.ax_3d.set_xlabel("X [m]")
                self.ax_3d.set_ylabel("Y [m]")
                self.ax_3d.set_zlabel("Z [m]")
                self.ax_3d.grid(True, alpha=0.3)
                self.plot_initialized = True
            
            colors = plt.cm.tab20(np.linspace(0, 1, len(self.robot_names)))
            
            for i, name in enumerate(self.robot_names):
                if len(self.drone_positions[name]) > 1:
                    positions = np.array(self.drone_positions[name])
                    
                    self.ax_3d.plot(positions[:,0], positions[:,1], positions[:,2], 
                                '-', color=colors[i], linewidth=1.5, alpha=0.7)
                    
                    if self.current_poses[name]:
                        self.ax_3d.scatter(
                            [self.current_poses[name][0]], 
                            [self.current_poses[name][1]], 
                            [self.current_poses[name][2]], 
                            color=colors[i], s=80, marker='o',
                            edgecolors='black', linewidth=0.5
                        )
            
            self.adjust_3d_axes_limits()
            self.canvas_3d.draw_idle()
            
        except Exception as e:
            self.get_logger().error(f"Error updating 3D plot: {str(e)}")

    def adjust_3d_axes_limits(self):
        all_positions = []
        for name in self.robot_names:
            if len(self.drone_positions[name]) > 0:
                all_positions.extend(self.drone_positions[name])
        
        if all_positions:
            positions = np.array(all_positions)
            min_vals = positions.min(axis=0)
            max_vals = positions.max(axis=0)
            
            margin = 0.1 * (max_vals - min_vals)
            self.ax_3d.set_xlim(min_vals[0]-margin[0], max_vals[0]+margin[0])
            self.ax_3d.set_ylim(min_vals[1]-margin[1], max_vals[1]+margin[1])
            self.ax_3d.set_zlim(min_vals[2]-margin[2], max_vals[2]+margin[2])

    def update_error_plot(self):
        if not self.time_history:
            return
            
        self.ax_error.clear()
        self.ax_error.plot(self.time_history, self.error_history['formation'], 'b-', label='Error de Formación')
        self.ax_error.set_title("Error de Formación")
        self.ax_error.set_xlabel("Tiempo [s]")
        self.ax_error.set_ylabel("Error [m]")
        self.ax_error.grid(True)
        self.ax_error.legend()
        
        self.canvas_error.draw()

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

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
        for name in self.robot_names:
            self.drone_positions[name].clear()
        self.error_history = {'formation': []}
        self.time_history = []
        if self.stack.currentIndex() == 0:
            self.update_3d_plot()
        elif self.stack.currentIndex() == 1:
            self.update_error_plot()
        self.publish_state(DroneState.RESTART_TRAJ)

    def show_warning(self, title, message):
        QMessageBox.warning(self, title, message)

    def closeEvent(self, event):
        self.cleanup()
        event.accept()
        
    def cleanup(self):
        self.publish_state(DroneState.IDLE)
        self.destroy_node()
        if hasattr(self, 'timer'):
            self.timer.stop()
        QApplication.quit()

    def update_current_view(self):
        current_time = time.time()
        if current_time - self.last_update_time < 0.1:
            return
            
        self.last_update_time = current_time
        
        if self.stack.currentIndex() == 0:
            self.update_3d_plot()
        elif self.stack.currentIndex() == 1:
            self.update_error_plot()

def sigint_handler(*args):
    QApplication.quit()

def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, sigint_handler)
    
    app = QApplication([])
    gui = BebopGUI()
    gui.show()

    gui.timer = QTimer()
    gui.timer.timeout.connect(lambda: rclpy.spin_once(gui, timeout_sec=0.01))
    gui.timer.start(10)

    app.aboutToQuit.connect(gui.cleanup)

    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    ret = app.exec_()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()