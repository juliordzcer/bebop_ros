class PID:
    def __init__(self, kp, kd, ki, min_output, max_output, name):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.min_output = min_output
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()
        self.pub_output = self.create_publisher(Float32, f'pid/output/{name}', 10)
        self.pub_error = self.create_publisher(Float32, f'pid/error/{name}', 10)
        self.pub_p = self.create_publisher(Float32, f'pid/p/{name}', 10)
        self.pub_d = self.create_publisher(Float32, f'pid/d/{name}', 10)
        self.pub_i = self.create_publisher(Float32, f'pid/i/{name}', 10)

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()

    def update(self, value, target_value):
        time = self.get_clock().now()
        dt = (time - self.previous_time).nanoseconds / 1e9
        error = target_value - value
        self.integral += error * dt
        p = self.kp * error
        d = 0.0
        if dt > 0:
            d = self.kd * (error - self.previous_error) / dt
        i = self.ki * self.integral
        output = p + d + i
        self.previous_error = error
        self.previous_time = time
        self.pub_output.publish(Float32(data=output))
        self.pub_error.publish(Float32(data=error))
        self.pub_p.publish(Float32(data=p))
        self.pub_d.publish(Float32(data=d))
        self.pub_i.publish(Float32(data=i))
        return max(min(output, self.max_output), self.min_output)
