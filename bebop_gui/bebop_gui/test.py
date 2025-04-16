#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class DroneStateSubscriber(Node):
    def __init__(self):
        super().__init__('drone_state_subscriber')
        
        # Diccionario de estados
        self.states = {
            0: 'IDLE',
            1: 'AUTOMATIC',
            2: 'TAKING_OFF',
            3: 'LANDING',
            4: 'EMERGENCY_STOP',
            5: 'HOVER',
            6: 'RESTART_TRAJ'
        }
        
        # Crear subscriber
        self.subscription = self.create_subscription(
            Int32,
            '/state',
            self.state_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        
        self.get_logger().info('Subscriber iniciado, esperando estados del dron...')
    
    def state_callback(self, msg):
        state = msg.data
        state_name = self.states.get(state, 'DESCONOCIDO')
        self.get_logger().info(f'Estado recibido: {state} - {state_name}')

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = DroneStateSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()