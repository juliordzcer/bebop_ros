from rclpy.node import Node
from std_msgs.msg import Float32

class PID:
    def __init__(self, node, kp, kd, ki, min_output, max_output, integrator_min, integrator_max, axis):
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

        # Log the initial parameters
        # self.node.get_logger().info(f'[{self.axis}] PID initialized with kp: {kp}, kd: {kd}, ki: {ki}, min_output: {min_output}, max_output: {max_output}')

    def reset(self):
        self.previous_error = 0.0
        self.integrator = 0.0

    def update(self, current_value, setpoint):
        error = setpoint - current_value
        derivative = error - self.previous_error
        self.integrator += error
        self.integrator = max(self.integrator_min, min(self.integrator, self.integrator_max))

        output = (self.kp * error) + (self.kd * derivative) + (self.ki * self.integrator)
        output = max(self.min_output, min(output, self.max_output))

        self.previous_error = error

        # Añadir logs para depuración
        # self.node.get_logger().info(f'[{self.axis}] Error: {error}, Output: {output}, Setpoint: {setpoint}, Current: {current_value}, kp: {self.kp}, kd: {self.kd}, ki: {self.ki}')

        return output