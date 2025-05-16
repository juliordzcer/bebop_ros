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
from std_msgs.msg import Float32MultiArray

import time

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel,
    QComboBox, QStackedWidget, QMessageBox, QHBoxLayout, QGridLayout, QDialog, QDialogButtonBox, QCheckBox,
)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QImage, QPixmap

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt  # Importación añadida

import matplotlib
matplotlib.use('Qt5Agg')  # Usar backend adecuado
matplotlib.rcParams['path.simplify'] = True
matplotlib.rcParams['path.simplify_threshold'] = 1.0


import cv2
from cv_bridge import CvBridge
from enum import IntEnum
from threading import Lock
import numpy as np
from collections import deque

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
        self.current_camera_view = "Todas las cámaras" 
        self.camera_states = {name: True for name in self.robot_names}  # Todas activas por defecto
        self.all_cameras_active = True  # Estado global de todas las cámaras
    
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
        self.bridge = CvBridge()
        self.camera_images = {name: None for name in self.robot_names}
        self.last_update_time = 0
        

    def setup_ros(self):
        qos = QoSProfile(depth=10)
        self.state_publisher = self.create_publisher(Int32, '/state', qos)
        
        # Suscriptores para cada dron
        self.pose_subs = []
        self.image_subs = []
        
        for name in self.robot_names:
            # Suscriptor de pose
            self.pose_subs.append(
                self.create_subscription(
                    Pose, f'/{name}/pose',
                    lambda msg, n=name: self.pose_callback(msg, n),
                    qos
                )
            )
            
            # Suscriptor de cámara
            self.image_subs.append(
                self.create_subscription(
                    Image, f'/world/bebop/model/{name}/link/body/sensor/rgb_camera_sensor/image',
                    lambda msg, n=name: self.image_callback(msg, n),
                    qos
                )
            )
        
        # Suscriptor de errores de formación (único error ahora)
        self.formation_error_sub = self.create_subscription(
            Float32MultiArray, '/formation_errors',
            self.formation_error_callback, qos
        )

    def setup_ui(self):
        self.setWindowTitle('Bebop Drone Controller')
        self.setFixedSize(1400, 1000)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        self.main_layout = QVBoxLayout()
        central_widget.setLayout(self.main_layout)

        # Título
        title = QLabel(f'Bebop Control Panel - {len(self.robot_names)} Drones')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 20px; font-weight: bold; color: #333;')
        self.main_layout.addWidget(title)

        # Botones
        self.setup_buttons()

        # Combo selectores
        self.setup_selectors()

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

        # Timer para actualización eficiente de vistas
        self.view_update_timer = QTimer()
        self.view_update_timer.timeout.connect(self.update_current_view)
        self.view_update_timer.start(10000)  # Actualizar cada 100ms (10 FPS)



    def setup_selectors(self):
        # Selector de vista principal
        self.view_selector = QComboBox()
        self.view_selector.addItems(["Vista de Cámaras", "Gráfica 3D", "Error de Formación"])
        self.view_selector.currentIndexChanged.connect(self.switch_view)
        self.main_layout.addWidget(self.view_selector)

        # Selector de cámara con estilo mejorado
        self.camera_selector = QComboBox()
        self.camera_selector.addItems(["Todas las cámaras"] + self.robot_names)
        self.camera_selector.currentIndexChanged.connect(self.change_camera_view)
        self.camera_selector.setVisible(False)
        self.camera_selector.setStyleSheet("""
            QComboBox {
                padding: 8px;
                font-size: 14px;
                min-width: 200px;
            }
        """)
        self.main_layout.addWidget(self.camera_selector)

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
        # Botones de control de cámaras
        camera_buttons = [
            ('Toggle All Cameras', self.toggle_all_cameras, '#2196F3'),
            ('Select Cameras to Disable', self.select_cameras_to_disable, '#673AB7')
        ]
        
        button_layout = QHBoxLayout()
        for text, callback, color in buttons:
            btn = QPushButton(text)
            btn.setStyleSheet(f"{button_style} QPushButton {{ background-color: {color}; color: white; }}")
            btn.clicked.connect(callback)
            button_layout.addWidget(btn)


        # Layout para botones de cámara
        camera_button_layout = QHBoxLayout()
        for text, callback, color in camera_buttons:
            btn = QPushButton(text)
            btn.setStyleSheet(f"{button_style} QPushButton {{ background-color: {color}; color: white; }}")
            btn.clicked.connect(callback)
            camera_button_layout.addWidget(btn)
        
        self.main_layout.addLayout(button_layout)
        self.main_layout.addLayout(camera_button_layout)

    def toggle_all_cameras(self):
        """Activa/desactiva todas las cámaras"""
        self.all_cameras_active = not self.all_cameras_active
        for name in self.robot_names:
            self.camera_states[name] = self.all_cameras_active
        
        if self.stack.currentIndex() == 0:  # Si estamos en la vista de cámaras
            self.update_camera_view_layout()
        
        status = "ON" if self.all_cameras_active else "OFF"
        self.show_warning("Camera Status", f"All cameras turned {status}")

    def select_cameras_to_disable(self):
        """Diálogo para seleccionar qué cámaras desactivar"""
        dialog = QDialog(self)
        dialog.setWindowTitle("Select Cameras to Disable")
        dialog.setMinimumWidth(300)
        
        layout = QVBoxLayout()
        
        # Crear checkboxes para cada dron
        self.camera_checkboxes = {}
        for name in self.robot_names:
            cb = QCheckBox(name)
            cb.setChecked(self.camera_states[name])  # Estado actual
            self.camera_checkboxes[name] = cb
            layout.addWidget(cb)
        
        # Botones de OK/Cancel
        btn_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btn_box.accepted.connect(dialog.accept)
        btn_box.rejected.connect(dialog.reject)
        layout.addWidget(btn_box)
        
        dialog.setLayout(layout)
        
        if dialog.exec_() == QDialog.Accepted:
            for name, cb in self.camera_checkboxes.items():
                self.camera_states[name] = cb.isChecked()
            
            self.update_camera_view_layout()
            self.show_warning("Camera Status", "Camera selection updated")
            
    def change_camera_view(self):
        self.current_camera_view = self.camera_selector.currentText()
        self.update_camera_view_layout()

    def update_camera_view_layout(self):
        # # Limpiar layout
        # for i in reversed(range(self.camera_layout.count())): 
        #     self.camera_layout.itemAt(i).widget().setParent(None)
        
        # if self.current_camera_view == "Todas las cámaras":
        #     # Mostrar todas las cámaras en grid
        #     num_drones = len(self.robot_names)
        #     cols = int(math.ceil(math.sqrt(num_drones)))
            
        #     for i, name in enumerate(self.robot_names):
        #         self.camera_layout.addWidget(self.camera_labels[name], i//cols, i%cols)
        # else:
        #     # Mostrar solo la cámara seleccionada (centrada)
        #     self.camera_layout.addWidget(self.camera_labels[self.current_camera_view], 0, 0, 2, 2)
        # Limpiar layout

        for i in reversed(range(self.camera_layout.count())): 
            self.camera_layout.itemAt(i).widget().setParent(None)
        
        if self.current_camera_view == "Todas las cámaras":
            # Mostrar solo cámaras activas en grid

            active_drones = [name for name in self.robot_names if self.camera_states[name]]
            num_active = len(active_drones)
            
            if num_active == 0:
                no_cam_label = QLabel("No cameras active")
                no_cam_label.setAlignment(Qt.AlignCenter)
                self.camera_layout.addWidget(no_cam_label, 0, 0)
                return
                
            cols = int(math.ceil(math.sqrt(num_active)))
            
            for i, name in enumerate(active_drones):
                self.camera_layout.addWidget(self.camera_labels[name], i//cols, i%cols)
        else:
            # Mostrar solo la cámara seleccionada si está activa
            if self.camera_states[self.current_camera_view]:
                self.camera_layout.addWidget(self.camera_labels[self.current_camera_view], 0, 0, 2, 2)
            else:
                disabled_label = QLabel(f"Camera {self.current_camera_view} is disabled")
                disabled_label.setAlignment(Qt.AlignCenter)
                self.camera_layout.addWidget(disabled_label, 0, 0, 2, 2)

    def toggle_all_cameras(self):
        """Activa/desactiva todas las cámaras"""
        self.all_cameras_active = not self.all_cameras_active
        # Actualiza todos los estados individuales
        for name in self.robot_names:
            self.camera_states[name] = self.all_cameras_active
        
        if self.stack.currentIndex() == 0:  # Si estamos en la vista de cámaras
            self.update_camera_view_layout()
        
        status = "ON" if self.all_cameras_active else "OFF"
        self.show_warning("Camera Status", f"All cameras turned {status}")

    def update_current_view(self):
        """Actualiza solo la vista activa para mejorar rendimiento"""
        current_time = time.time()
        if current_time - self.last_update_time < 0.001:  # Limitar a 10 FPS
            return
            
        self.last_update_time = current_time
        
        if self.stack.currentIndex() == 0:  # Vista de cámaras
            self.update_camera_images()
        elif self.stack.currentIndex() == 1:  # Gráfica 3D
            self.update_3d_plot()
        elif self.stack.currentIndex() == 2:  # Gráfica de error
            self.update_error_plot()

    def update_camera_images(self):
        # """Actualiza solo las imágenes visibles"""
        # if self.current_camera_view == "Todas las cámaras":
        #     for name in self.robot_names:
        #         if self.camera_images[name] is not None:
        #             self.display_image(self.camera_images[name], self.camera_labels[name])
        # else:
        #     name = self.current_camera_view
        #     if self.camera_images[name] is not None:
        #         self.display_image(self.camera_images[name], self.camera_labels[name])
        """Actualiza solo las imágenes visibles y activas"""
        if self.current_camera_view == "Todas las cámaras":
            for name in self.robot_names:
                if self.camera_states[name] and self.camera_images[name] is not None:
                    self.display_image(self.camera_images[name], self.camera_labels[name])
        else:
            name = self.current_camera_view
            if self.camera_states[name] and self.camera_images[name] is not None:
                self.display_image(self.camera_images[name], self.camera_labels[name])

    def setup_camera_view(self):
        self.camera_container = QWidget()
        self.camera_layout = QGridLayout()
        self.camera_container.setLayout(self.camera_layout)
        
        # Crear etiquetas para cada cámara
        self.camera_labels = {}
        
        for name in self.robot_names:
            label = QLabel()
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumSize(320, 240)
            label.setStyleSheet("""
                QLabel {
                    border: 2px solid #ddd;
                    background-color: #f0f0f0;
                }
            """)
            self.camera_labels[name] = label
        
        self.update_camera_view_layout()
        self.stack.addWidget(self.camera_container)

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
        self.camera_selector.setVisible(index == 0)  # Mostrar selector solo en vista de cámaras
        
        if index == 1:  # Gráfica 3D
            self.update_3d_plot()
        elif index == 2:  # Gráfica de error
            self.update_error_plot()

    def update_camera_view(self):
        selected = self.camera_selector.currentText()
        
        # Ocultar todas las cámaras
        for label in self.camera_labels.values():
            label.setVisible(False)
        
        if selected == "Todas las cámaras":
            # Mostrar todas las cámaras en grid
            num_drones = len(self.robot_names)
            cols = int(math.ceil(math.sqrt(num_drones)))
            
            for i, (name, label) in enumerate(self.camera_labels.items()):
                label.setVisible(True)
                self.camera_layout.addWidget(label, i//cols, i%cols)
        else:
            # Mostrar solo la cámara seleccionada (centrada)
            label = self.camera_labels[selected]
            label.setVisible(True)
            self.camera_layout.addWidget(label, 0, 0, 2, 2)

    def image_callback(self, msg, drone_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.camera_images[drone_name] = cv_image
            
            # Solo actualizar si estamos en la vista de cámaras y esta cámara es visible
            if self.stack.currentIndex() == 0:
                selected = self.camera_selector.currentText()
                if selected == "Todas las cámaras" or selected == drone_name:
                    self.display_image(cv_image, self.camera_labels[drone_name])
                    
        except Exception as e:
            self.get_logger().error(f"Error en image_callback para {drone_name}: {str(e)}")

    def display_image(self, cv_image, label):
        try:
            h, w = cv_image.shape[:2]
            ratio = min(320/w, 240/h)
            cv_image = cv2.resize(cv_image, (int(w*ratio), int(h*ratio)))
            
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            q_img = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            
            label.setPixmap(QPixmap.fromImage(q_img))
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {str(e)}")

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
            if self.stack.currentIndex() == 1:
                self.update_3d_plot()
                
        except Exception as e:
            self.get_logger().error(f"Error processing {drone_name} pose: {str(e)}")

    def formation_error_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        
        if len(msg.data) == len(self.robot_names):
            # Calcular error promedio de formación
            avg_error = sum(msg.data) / len(msg.data)
            self.error_history['formation'].append(avg_error)
            self.time_history.append(current_time)
            
            # Limitar el historial
            max_history = 1000
            if len(self.time_history) > max_history:
                self.time_history = self.time_history[-max_history:]
                self.error_history['formation'] = self.error_history['formation'][-max_history:]
            
            # Actualizar gráfica si está visible
            if self.stack.currentIndex() == 2:
                self.update_error_plot()

    def update_3d_plot(self):
        try:
            # No limpiar completamente el gráfico para mejor rendimiento
            self.ax_3d.cla()  # Más eficiente que clear()
            
            # Configuración inicial del gráfico (solo una vez)
            if not hasattr(self, 'plot_initialized'):
                self.ax_3d.set_title(f"3D Trajectory - {len(self.robot_names)} Drones")
                self.ax_3d.set_xlabel("X [m]")
                self.ax_3d.set_ylabel("Y [m]")
                self.ax_3d.set_zlabel("Z [m]")
                self.ax_3d.grid(True, alpha=0.3)
                self.plot_initialized = True
            
            # Pre-alocar arrays para mejor performance
            colors = plt.cm.tab20(np.linspace(0, 1, len(self.robot_names)))
            
            for i, name in enumerate(self.robot_names):
                if len(self.drone_positions[name]) > 1:
                    positions = np.array(self.drone_positions[name])
                    
                    # Dibujar trayectoria con línea sólida
                    self.ax_3d.plot(positions[:,0], positions[:,1], positions[:,2], 
                                '-', color=colors[i], linewidth=1.5, alpha=0.7)
                    
                    # Dibujar posición actual como marcador
                    if self.current_poses[name]:
                        self.ax_3d.scatter(
                            [self.current_poses[name][0]], 
                            [self.current_poses[name][1]], 
                            [self.current_poses[name][2]], 
                            color=colors[i], s=80, marker='o',
                            edgecolors='black', linewidth=0.5
                        )
            
            # Ajustar límites dinámicamente
            self.adjust_3d_axes_limits()
            
            # Dibujar solo si hay cambios significativos
            self.canvas_3d.draw_idle()  # Más eficiente que draw()
            
        except Exception as e:
            self.get_logger().error(f"Error updating 3D plot: {str(e)}")

    def adjust_3d_axes_limits(self):
        """Ajusta los límites de los ejes dinámicamente"""
        all_positions = []
        for name in self.robot_names:
            if len(self.drone_positions[name]) > 0:
                all_positions.extend(self.drone_positions[name])
        
        if all_positions:
            positions = np.array(all_positions)
            min_vals = positions.min(axis=0)
            max_vals = positions.max(axis=0)
            
            # Añadir margen del 10%
            margin = 0.1 * (max_vals - min_vals)
            self.ax_3d.set_xlim(min_vals[0]-margin[0], max_vals[0]+margin[0])
            self.ax_3d.set_ylim(min_vals[1]-margin[1], max_vals[1]+margin[1])
            self.ax_3d.set_zlim(min_vals[2]-margin[2], max_vals[2]+margin[2])

    def update_error_plot(self):
        if not self.time_history:
            return
            
        # Actualizar gráfica de error de formación
        self.ax_error.clear()
        self.ax_error.plot(self.time_history, self.error_history['formation'], 'b-', label='Error de Formación')
        self.ax_error.set_title("Error de Formación")
        self.ax_error.set_xlabel("Tiempo [s]")
        self.ax_error.set_ylabel("Error [m]")
        self.ax_error.grid(True)
        self.ax_error.legend()
        
        self.canvas_error.draw()

    def quaternion_to_yaw(self, q):
        # Convertir cuaternión a ángulo de yaw (en radianes)
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
    
    # Número de drones (puede pasarse como argumento)
    # num_drones = 20  # Puedes cambiar este valor
    
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