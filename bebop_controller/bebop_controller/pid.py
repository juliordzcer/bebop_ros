# Copyright 2025 Julio César Rodríguez
# Licensed under the Apache License, Version 2.0
# https://www.apache.org/licenses/LICENSE-2.0


from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class PID:
    def __init__(self, node, kp, kd, ki, min_output, max_output, integrator_min, integrator_max, dt, axis, debug=False):
        self.node = node
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.min_output = min_output
        self.max_output = max_output
        self.integrator_min = integrator_min
        self.integrator_max = integrator_max
        self.axis = axis
        self.previous_error = 0.0
        self.integrator = 0.0
        self.dt = dt
        self.debug = debug  # Nuevo flag para depuración

        if self.debug:
            self.node.get_logger().info(
                f'[{self.axis}] PID initialized with kp: {kp}, kd: {kd}, ki: {ki}, '
                f'min_output: {min_output}, max_output: {max_output}'
            )

    def reset(self):
        self.previous_error = 0.0
        self.integrator = 0.0

    def update(self, current_value, setpoint):
        error = setpoint - current_value
        
        # Protección contra división por cero en la derivada
        derivative = (error - self.previous_error) / self.dt if self.dt > 1e-6 else 0.0

        # Actualizar el integrador con saturación
        self.integrator += error * self.dt
        self.integrator = np.clip(self.integrator, self.integrator_min, self.integrator_max)

        # Calcular salida del PID
        output = (self.kp * error) + (self.kd * derivative) + (self.ki * self.integrator)
        output = np.clip(output, self.min_output, self.max_output)

        self.previous_error = error

        if self.debug:
            self.node.get_logger().info(
                f'[{self.axis}] Error: {error}, Output: {output}, Setpoint: {setpoint}, '
                f'Current: {current_value}, kp: {self.kp}, kd: {self.kd}, ki: {self.ki}'
            )

        return output