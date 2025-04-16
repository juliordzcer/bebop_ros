#!/usr/bin/env python3

import rclpy
import math
from enum import IntEnum
from threading import Lock
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                             QVBoxLayout, QWidget, QLabel, QMessageBox)
from PyQt5.QtCore import Qt, QTimer
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class DroneState(IntEnum):
    IDLE = 0
    AUTOMATIC = 1
    TAKING_OFF = 2
    LANDING = 3
    EMERGENCY_STOP = 4
    HOVER = 5
    RESTART_TRAJ = 6

class BebopGUI(Node, QMainWindow):
    def __init__(self):
        # ROS Initialization
        Node.__init__(self, 'bebop_gui')
        self.setup_ros()
        
        # Qt Initialization
        QMainWindow.__init__(self)
        self.setup_ui()
        
        # State management
        self.state_lock = Lock()
        self.current_state = DroneState.IDLE
        self.goal_positions = []

    def setup_ros(self):
        qos = QoSProfile(depth=10)
        self.state_publisher = self.create_publisher(Int32, '/state', qos)
        self.goal_subscription = self.create_subscription(
            Pose, '/goal', self.goal_callback, qos)
        self.get_logger().info("ROS components initialized")

    def setup_ui(self):
        self.setWindowTitle('Bebop Drone Controller')
        self.setFixedSize(600, 700)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        # Title
        title = QLabel('Bebop Control Panel')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 20px; font-weight: bold; color: #333;')
        layout.addWidget(title)
        
        # Buttons
        self.setup_buttons(layout)
        
        # Status
        self.status_label = QLabel('Status: IDLE')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet('font-size: 16px; color: #555;')
        layout.addWidget(self.status_label)

        # 3D Plot
        self.setup_plot()
        layout.addWidget(self.canvas)

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
            ('Hover', self.hover, '#2196F3'),
            ('EMERGENCY STOP', self.emergency_stop, '#d32f2f'),
            ('Reset Trajectory', self.restart_trajectory, '#9C27B0')
        ]
        
        for text, callback, color in buttons:
            btn = QPushButton(text)
            btn.setStyleSheet(f"{button_style} QPushButton {{ background-color: {color}; color: white; }}")
            btn.clicked.connect(callback)
            layout.addWidget(btn)

    def setup_plot(self):
        self.figure = Figure(figsize=(5, 4), tight_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')
        
        # Professional plot styling
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.xaxis._axinfo["grid"].update({"linewidth":0.5})
        self.ax.set_title("3D Trajectory", pad=20)
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Z [m]")

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
            DroneState.HOVER: "HOVER",
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

    def update_plot(self):
        try:
            self.ax.clear()
            if self.goal_positions:
                x, y, z = zip(*self.goal_positions)
                self.ax.plot(x, y, z, 'b-o', linewidth=2, markersize=5)
                self.ax.scatter(x[-1], y[-1], z[-1], c='r', s=100, marker='*')
            
            self.ax.set_title("3D Trajectory", pad=20)
            self.ax.set_xlabel("X [m]")
            self.ax.set_ylabel("Y [m]")
            self.ax.set_zlabel("Z [m]")
            self.canvas.draw()
        except Exception as e:
            self.get_logger().error(f"Error updating plot: {str(e)}")

    # Button callbacks
    def automatic(self): self.publish_state(DroneState.AUTOMATIC)
    def takeoff(self): self.publish_state(DroneState.TAKING_OFF)
    def land(self): self.publish_state(DroneState.LANDING)
    def hover(self): self.publish_state(DroneState.HOVER)
    def emergency_stop(self): self.publish_state(DroneState.EMERGENCY_STOP)
    def restart_trajectory(self):
        self.goal_positions.clear()
        self.update_plot()
        self.publish_state(DroneState.RESTART_TRAJ)

    def show_warning(self, title, message):
        QMessageBox.warning(self, title, message)
    
    def closeEvent(self, event):
        self.destroy_node()
        event.accept()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])

    gui = None  # por si ocurre un error antes de que se instancie
    
    try:
        gui = BebopGUI()
        gui.show()

        # QTimer para integrar el spin de ROS y detectar cierre
        timer = QTimer()

        def spin_and_check():
            if rclpy.ok():
                rclpy.spin_once(gui, timeout_sec=0.01)
            else:
                gui.close()  # cerrar GUI si ROS se apagó

        timer.timeout.connect(spin_and_check)
        timer.start(10)

        app.exec_()
        timer.stop()

    except Exception as e:
        print(f"Fatal error: {str(e)}")

    finally:
        if gui is not None and hasattr(gui, 'destroy_node'):
            gui.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
