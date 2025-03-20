from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class STA:
    def __init__(self, node, k0, k1, k2, min_output, max_output, integrator_min, integrator_max, dt, axis):
        self.node = node
        self.k0 = k0
        self.k1 = k1
        self.k2 = k2
        self.min_output = min_output
        self.max_output = max_output
        self.integrator_min = integrator_min
        self.integrator_max = integrator_max
        self.axis = axis
        self.previous_error = 0.0
        self.integrator = 0.0
        self.dt = dt
        self.s = 0.0
        self.nu = 0.0

    def reset(self):
        self.previous_error = 0.0
        self.integrator = 0.0
        self.s = 0.0
        self.nu = 0.0

    def update(self, current_value, setpoint):
        error = current_value - setpoint
        derror = (error - self.previous_error) / self.dt

        # Ajuste en la ecuación de 's'
        self.s = derror + self.k0 * error

        # Integración con control de saturación
        self.nu += - self.k2 * np.sign(self.s) * self.dt
        self.nu = np.clip(self.nu, self.integrator_min, self.integrator_max)

        # Cálculo de la salida con STA
        output =  - self.k1 * np.power(np.abs(self.s), 0.5) * np.sign(self.s) + self.nu
        output = np.clip(output, self.min_output, self.max_output)

        self.previous_error = error
        return output  
