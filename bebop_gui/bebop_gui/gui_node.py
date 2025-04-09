#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                            QVBoxLayout, QWidget, QLabel, QMessageBox)
from PyQt5.QtCore import Qt
from std_msgs.msg import Int32

class BebopGUI(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, 'bebop_gui')
        QMainWindow.__init__(self)
        
        self.setWindowTitle('Bebop Drone Controller')
        self.setFixedSize(300, 400)
        
        # State constants
        self.IDLE = 0
        self.AUTOMATIC = 1
        self.TAKING_OFF = 2
        self.LANDING = 3
        self.EMERGENCY_STOP = 4
        self.HOVER = 5
        
        # Configuración de ROS 2 - Publisher
        self.state_publisher = self.create_publisher(Int32, '/drone_state', 10)
        
        # Interfaz gráfica
        self.init_ui()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        # Título
        title = QLabel('Bebop Control')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('font-size: 18px; font-weight: bold;')
        layout.addWidget(title)
        
        # Botones
        self.automatic_btn = self.create_button('Automatic', self.automatic, '#FF9800')
        self.takeoff_btn = self.create_button('Take Off', self.takeoff, '#4CAF50')
        self.land_btn = self.create_button('Land', self.land, '#f44336')
        self.emergency_btn = self.create_button('EMERGENCY STOP', self.emergency_stop, '#d32f2f')
        self.hover_btn = self.create_button('Hover', self.hover, '#2196F3')
        
        # Añadir botones al layout
        layout.addWidget(self.automatic_btn)
        layout.addWidget(self.takeoff_btn)
        layout.addWidget(self.land_btn)
        layout.addWidget(self.emergency_btn)
        layout.addWidget(self.hover_btn)
        
        # Estado del dron
        self.status_label = QLabel('Status: Disconnected')
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
    def create_button(self, text, callback, color):
        btn = QPushButton(text)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                padding: 10px;
                font-size: 14px;
                border-radius: 5px;
                margin: 5px;
            }}
            QPushButton:hover {{
                background-color: #555;
            }}
            QPushButton:pressed {{
                background-color: #777;
            }}
        """)
        btn.clicked.connect(callback)
        return btn
    
    def publish_state(self, state):
        msg = Int32()
        msg.data = state
        self.state_publisher.publish(msg)
    
    def automatic(self):
        self.publish_state(self.AUTOMATIC)
        self.status_label.setText('Status: Automatic')
        
    def takeoff(self):
        self.publish_state(self.TAKING_OFF)
        self.status_label.setText('Status: Taking off...')
        
    def land(self):
        self.publish_state(self.LANDING)
        self.status_label.setText('Status: Landing...')
    
    def hover(self):
        self.publish_state(self.HOVER)
        self.status_label.setText('Status: Hover')
    
    def emergency_stop(self):
        self.publish_state(self.EMERGENCY_STOP)
        self.status_label.setText('Status: Emergency Stopped')
    
    def show_warning(self, title, message):
        QMessageBox.warning(self, title, message)
    
    def closeEvent(self, event):
        self.destroy_node()
        event.accept()

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication([])
    gui = BebopGUI()
    gui.show()
    
    # Ejecutar el bucle de eventos de ROS en un hilo separado
    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(gui,))
    ros_thread.start()
    
    app.exec_()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()