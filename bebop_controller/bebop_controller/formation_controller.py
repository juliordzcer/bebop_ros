#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool, Int32, Float32MultiArray
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion
import numpy as np
import json
from collections import deque
from enum import IntEnum
import math

class PID:
    def __init__(self, node, kp, kd, ki, min_output, max_output, integrator_min, integrator_max, dt, axis):
        self.node = node
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.min_output = min_output
        self.max_output = max_output
        self.integrator_min = integrator_min
        self.integrator_max = integrator_max
        self.dt = dt
        self.axis = axis
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, current, target):
        error = target - current
        
        # Proportional term
        p_term = self.kp * error
        
        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        
        # Integral term with anti-windup
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, self.integrator_min, self.integrator_max)
        i_term = self.ki * self.integral
        
        # Calculate output
        output = p_term + i_term + d_term
        output = np.clip(output, self.min_output, self.max_output)
        
        self.prev_error = error
        return output

class FormationController(Node):
    class State(IntEnum):
        IDLE = 0
        AUTOMATIC = 1
        TAKING_OFF = 2
        LANDING = 3
        EMERGENCY_STOP = 4

    def __init__(self):
        super().__init__('formation_controller')
        # Configuration parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('robot_names', '["bebop1", "bebop2", "bebop3", "bebop4"]'),
                ('frequency', 100.0),
                ('takeoff_height', 2.5),
                ('min_altitude', 0.5),
                ('takeoff_threshold', 0.05),
                ('landing_threshold', 0.08),
                ('formation_scale', 1.5),
                ('pyramid_base_width', 3.0),
                ('pyramid_height_factor', 1.5),
                ('max_linear_velocity', 1.5),
                ('max_angular_velocity', 3.0),
                ('trajectory_speed', 0.4),
                ('safety_margin', 0.5)
            ])

        # Get parameters
        self.robot_names = json.loads(self.get_parameter('robot_names').value)
        self.frequency = self.get_parameter('frequency').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.min_altitude = self.get_parameter('min_altitude').value
        self.takeoff_threshold = self.get_parameter('takeoff_threshold').value
        self.landing_threshold = self.get_parameter('landing_threshold').value
        self.formation_scale = self.get_parameter('formation_scale').value
        self.pyramid_base_width = self.get_parameter('pyramid_base_width').value
        self.pyramid_height_factor = self.get_parameter('pyramid_height_factor').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.trajectory_speed = self.get_parameter('trajectory_speed').value
        self.safety_margin = self.get_parameter('safety_margin').value

        # Initialize data structures
        self.robot_poses = {name: None for name in self.robot_names}
        self.robot_orientations = {name: None for name in self.robot_names}
        self.robot_velocities = {name: np.zeros(3) for name in self.robot_names}
        self.pose_history = {name: deque(maxlen=5) for name in self.robot_names}
        self.last_pose_time = {name: None for name in self.robot_names}
        
        # Trajectory tracking variables
        self.leader_position = np.zeros(3)
        self.leader_yaw = 0.0
        self.automatic_start_time = None

        # Generate pyramid formation automatically
        self.relative_positions = self.generate_pyramid_formation(len(self.robot_names))
        
        # Setup ROS communication
        qos = QoSProfile(depth=10)
        
        # Publishers
        self.cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', qos)
            for name in self.robot_names
        }
        self.enable_pubs = {
            name: self.create_publisher(Bool, f'/{name}/enable', qos)
            for name in self.robot_names
        }

        self.formation_error_pub = self.create_publisher(Float32MultiArray, '/formation_errors', qos)
        self.desired_positions = np.zeros((len(self.robot_names), 3))

        # Subscribers
        self.pose_subs = [
            self.create_subscription(
                Pose, f'/{name}/pose',
                lambda msg, name=name: self.pose_callback(msg, name),
                qos
            ) for name in self.robot_names
        ]
        self.state_sub = self.create_subscription(
            Int32, '/state', self.state_changed, qos
        )

        # Services
        self.takeoff_srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(Empty, 'land', self.land_callback)

        # Initialize PID controllers
        dt = 1.0 / self.frequency
        self.pids = {
            name: {
                'x': PID(self, 1.5, 0.05, 0.2, -1.0, 1.0, -0.5, 0.5, dt, 'x'),
                'y': PID(self, 1.2, 0.04, 0.12, -1.0, 1.0, -0.5, 0.5, dt, 'y'),
                'z': PID(self, 1.2, 0.04, 0.12, -1.0, 1.0, -0.2, 0.2, dt, 'z'),
                'yaw': PID(self, 0.8, 0.3, 0.05, -1.0, 1.0, -0.5, 0.5, dt, 'yaw')
            }
            for name in self.robot_names
        }

        # State variables
        self.state = self.State.IDLE
        self.enable = False
        self.takeoff_complete = False

        # Control timer
        self.control_timer = self.create_timer(dt, self.control_loop)

    def generate_pyramid_formation(self, num_agents):
        """Generate pyramid formation positions for the given number of agents"""
        positions = np.zeros((num_agents, 3))
        
        if num_agents == 0:
            return positions
        
        # Configuration parameters
        leader_height = self.takeoff_height * self.pyramid_height_factor
        layer_height_reduction = self.takeoff_height * self.safety_margin  # Cada capa 30% más baja
        xy_separation = self.pyramid_base_width / 2.0
        
        # Leader position (always at top)
        positions[0] = [0, 0, leader_height]
        
        if num_agents == 1:
            return positions
        
        # Calculate how many full layers we can have
        remaining_agents = num_agents - 1
        layer_agents = 4  # First layer after leader has 4 agents
        layers = []
        
        while remaining_agents > 0:
            agents_in_layer = min(layer_agents, remaining_agents)
            layers.append(agents_in_layer)
            remaining_agents -= agents_in_layer
            layer_agents += 4  # Each subsequent layer adds 4 more agents
        
        # Generate positions for each layer
        agent_index = 1
        
        for layer_num, agents_in_layer in enumerate(layers, 1):
            layer_z = leader_height - (layer_num * layer_height_reduction)
            layer_radius = layer_num * xy_separation
            
            # Calculate angle step between agents
            angle_step = 2 * math.pi / agents_in_layer
            
            for i in range(agents_in_layer):
                angle = i * angle_step
                x = layer_radius * math.cos(angle)
                y = layer_radius * math.sin(angle)
                
                # For even layers, offset radius slightly for better distribution
                if layer_num % 2 == 0:
                    x *= 1.2
                    y *= 1.2
                
                positions[agent_index] = [x, y, layer_z]
                agent_index += 1
        
        return positions
    
    def calculate_formation_errors(self, positions):
        """Calcula los errores de posición relativa respecto a la formación deseada"""
        n = len(self.robot_names)
        formation_errors = np.zeros(n)
        
        # Return zeros if desired_positions hasn't been set yet
        if not hasattr(self, 'desired_positions'):
            return formation_errors
            
        for i in range(n):
            error_sum = 0.0
            count = 0
            
            for j in range(n):
                if i != j:
                    # Error de posición relativa
                    desired_rel_pos = self.desired_positions[j] - self.desired_positions[i]
                    actual_rel_pos = positions[j] - positions[i]
                    
                    error_sum += np.linalg.norm(actual_rel_pos - desired_rel_pos)
                    count += 1
                    
            if count > 0:
                formation_errors[i] = error_sum / count
        return formation_errors

    def pose_callback(self, msg, robot_name):
        """Update pose and estimate velocity"""
        try:
            current_time = self.get_clock().now()
            new_pose = np.array([msg.position.x, msg.position.y, msg.position.z])
            
            # Update orientation
            orientation_q = msg.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.robot_orientations[robot_name] = euler_from_quaternion(orientation_list)
            
            # Update leader reference if this is the leader
            if robot_name == self.robot_names[0]:
                self.leader_position = new_pose
                self.leader_yaw = self.robot_orientations[robot_name][2]
            
            # Estimate velocity
            if self.last_pose_time[robot_name] is not None:
                dt = (current_time - self.last_pose_time[robot_name]).nanoseconds * 1e-9
                if dt > 0 and self.robot_poses[robot_name] is not None:
                    velocity = (new_pose - self.robot_poses[robot_name]) / dt
                    self.pose_history[robot_name].append(velocity)
                    if len(self.pose_history[robot_name]) > 0:
                        self.robot_velocities[robot_name] = np.mean(self.pose_history[robot_name], axis=0)
            
            self.robot_poses[robot_name] = new_pose
            self.last_pose_time[robot_name] = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error in pose callback for {robot_name}: {str(e)}")


    def pose_callback(self, msg, robot_name):
        """Update pose and estimate velocity"""
        try:
            current_time = self.get_clock().now()
            new_pose = np.array([msg.position.x, msg.position.y, msg.position.z])
            
            # Update orientation
            orientation_q = msg.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.robot_orientations[robot_name] = euler_from_quaternion(orientation_list)
            
            # Update leader reference if this is the leader
            if robot_name == self.robot_names[0]:
                self.leader_position = new_pose
                self.leader_yaw = self.robot_orientations[robot_name][2]
            
            # Estimate velocity
            if self.last_pose_time[robot_name] is not None:
                dt = (current_time - self.last_pose_time[robot_name]).nanoseconds * 1e-9
                if dt > 0 and self.robot_poses[robot_name] is not None:
                    velocity = (new_pose - self.robot_poses[robot_name]) / dt
                    self.pose_history[robot_name].append(velocity)
                    if len(self.pose_history[robot_name]) > 0:
                        self.robot_velocities[robot_name] = np.mean(self.pose_history[robot_name], axis=0)
            
            self.robot_poses[robot_name] = new_pose
            self.last_pose_time[robot_name] = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error in pose callback for {robot_name}: {str(e)}")

    def state_changed(self, msg):
        """Handle state changes"""
        new_state = msg.data
        
        if new_state == self.State.IDLE and self.state != self.State.IDLE:
            self.get_logger().info("Changing to IDLE state")
            self.state = self.State.IDLE
            self.enable = False
            self.automatic_start_time = None
                
        elif new_state == self.State.AUTOMATIC:
            if self.state == self.State.TAKING_OFF and self.takeoff_complete:
                self.get_logger().info("Changing to AUTOMATIC state")
                self.state = self.State.AUTOMATIC
                self.enable = True
                self.automatic_start_time = self.get_clock().now()
            elif self.state != self.State.TAKING_OFF:
                self.get_logger().warn("Cannot switch to AUTOMATIC without completing takeoff first")
                
        elif new_state == self.State.TAKING_OFF and self.state != self.State.TAKING_OFF:
            self.get_logger().info("Changing to TAKING_OFF state")
            self.state = self.State.TAKING_OFF
            self.takeoff_complete = False
            self.enable = True
                
        elif new_state == self.State.LANDING and self.state != self.State.LANDING:
            self.get_logger().info("Changing to LANDING state")
            self.state = self.State.LANDING
            self.enable = True
                
        elif new_state == self.State.EMERGENCY_STOP and self.state != self.State.EMERGENCY_STOP:
            self.get_logger().warn("EMERGENCY STOP activated!")
            self.state = self.State.EMERGENCY_STOP
            self.enable = False
        
        # Publish enable state to all robots
        self.publish_enable_state()

    def publish_enable_state(self):
        """Publish current enable state to all robots"""
        enable_msg = Bool(data=self.enable)
        for pub in self.enable_pubs.values():
            pub.publish(enable_msg)

    def takeoff_callback(self, request, response):
        self.get_logger().info('Takeoff requested!')
        if self.state != self.State.TAKING_OFF:
            self.state = self.State.TAKING_OFF
            self.takeoff_complete = False
            self.enable = True
            self.publish_enable_state()
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Landing requested!')
        if self.state != self.State.LANDING:
            self.state = self.State.LANDING
            self.enable = True
            self.publish_enable_state()
        return response

    def control_loop(self):
        try:
            if self.state == self.State.TAKING_OFF:
                self.handle_takeoff()
            elif self.state == self.State.LANDING:
                self.handle_landing()
            elif self.state in [self.State.IDLE, self.State.EMERGENCY_STOP]:
                self.handle_stop_state()
            elif self.state == self.State.AUTOMATIC:
                self.automatic_state()
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {str(e)}")
            self.enable = False
            self.publish_enable_state()

    def handle_takeoff(self):
        """Handle takeoff control logic"""
        all_reached = True
        
        for i, name in enumerate(self.robot_names):
            if self.robot_poses[name] is None:
                all_reached = False
                continue
                
            current_z = self.robot_poses[name][2]
            target_z = self.takeoff_height + self.relative_positions[i][2]
            
            # Ensure minimum altitude
            target_z = max(target_z, self.min_altitude)
            
            if current_z < target_z - self.takeoff_threshold:
                msg = Twist()
                msg.linear.z = float(self.pids[name]['z'].update(current_z, target_z))
                self.cmd_pubs[name].publish(msg)
                all_reached = False
            else:
                # Maintain altitude
                msg = Twist()
                msg.linear.z = float(self.pids[name]['z'].update(current_z, target_z))
                self.cmd_pubs[name].publish(msg)
        
        if all_reached:
            self.get_logger().info("All drones reached takeoff height")
            self.takeoff_complete = True

    def handle_landing(self):
        """Handle landing control logic with altitude safety"""
        all_landed = True
        
        for i, name in enumerate(self.robot_names):
            if self.robot_poses[name] is None:
                all_landed = False
                continue
                
            current_z = self.robot_poses[name][2]
            target_z = 0.05  # Just above ground
            
            # Calculate safe descent rate based on position in formation
            max_descent = 0.3 + (0.1 * i)  # Lower drones descend slower
            
            if current_z > self.landing_threshold:
                msg = Twist()
                # Use PID but limit descent rate
                z_control = self.pids[name]['z'].update(current_z, target_z)
                z_control = np.clip(z_control, -max_descent, max_descent)
                msg.linear.z = float(z_control)
                self.cmd_pubs[name].publish(msg)
                all_landed = False
            else:
                self.get_logger().info(f"Landing completed for {name}!")
                self.cmd_pubs[name].publish(Twist())
        
        if all_landed:
            self.state = self.State.IDLE
            self.enable = False
            self.publish_enable_state()

    def handle_stop_state(self):
        """Handle IDLE and EMERGENCY states"""
        self.enable = False
        self.publish_enable_state()
        for pub in self.cmd_pubs.values():
            pub.publish(Twist())

    def automatic_state(self):
        """Implement formation control with altitude safety"""
        try:
            # Check for missing data
            if any(pos is None for pos in self.robot_poses.values()):
                self.get_logger().warn("Missing pose data", throttle_duration_sec=1.0)
                return

            # Get elapsed time
            current_time = self.get_clock().now()
            if self.automatic_start_time is None:
                self.automatic_start_time = current_time
            elapsed_time = (current_time - self.automatic_start_time).nanoseconds * 1e-9

            n = len(self.robot_names)
            commands = np.zeros((n, 3))
            cyaw = np.zeros(n)

            positions = np.array([self.robot_poses[name] for name in self.robot_names])

            # Calcular errores
            formation_errors = self.calculate_formation_errors(positions)
            
            # Publicar errores de formación
            formation_msg = Float32MultiArray()
            formation_msg.data = formation_errors.tolist()
            self.formation_error_pub.publish(formation_msg)


            # First calculate all desired positions
            self.desired_positions = np.zeros((n, 3))
            for i in range(n):
                if i == 0:  # Leader
                    self.desired_positions[i][0] = min(elapsed_time * self.trajectory_speed, 10.0)
                    self.desired_positions[i][1] = 0.0
                    self.desired_positions[i][2] = self.takeoff_height
                else:  # Followers
                    # Get relative position from formation pattern
                    rel_pos = self.relative_positions[i]
                    
                    # Rotate relative position by leader's yaw
                    yaw = self.leader_yaw
                    x_rel = rel_pos[0] * math.cos(yaw) - rel_pos[1] * math.sin(yaw)
                    y_rel = rel_pos[0] * math.sin(yaw) + rel_pos[1] * math.cos(yaw)
                    z_rel = rel_pos[2]
                    
                    # Calculate absolute desired position
                    self.desired_positions[i][0] = self.leader_position[0] + x_rel
                    self.desired_positions[i][1] = self.leader_position[1] + y_rel
                    
                    # Calculate desired altitude with safety constraints
                    desired_altitude = self.leader_position[2] + z_rel
                    min_allowed = self.min_altitude + (abs(z_rel) * 0.5)  # Higher layers have more margin
                    self.desired_positions[i][2] = max(desired_altitude, min_allowed)

            # Then calculate commands for each drone
            for i, name in enumerate(self.robot_names):
                if self.robot_poses[name] is None or self.robot_orientations[name] is None:
                    continue
                
                current_pos = self.robot_poses[name]
                euler_c = self.robot_orientations[name]
                target_pos = self.desired_positions[i]
                
                # Calculate position error with safety checks
                pos_error = target_pos - current_pos
                
                # Limit XY velocity based on altitude to prevent crashes
                altitude_factor = max(0.2, min(1.0, current_pos[2] / self.takeoff_height))
                max_vel_xy = self.max_linear_velocity * altitude_factor
                
                commands[i, 0] = float(np.clip(
                    self.pids[name]['x'].update(current_pos[0], target_pos[0]),
                    -max_vel_xy, max_vel_xy
                ))
                commands[i, 1] = float(np.clip(
                    self.pids[name]['y'].update(current_pos[1], target_pos[1]),
                    -max_vel_xy, max_vel_xy
                ))
                
                # Special handling for Z axis to prevent ground collisions
                z_control = self.pids[name]['z'].update(current_pos[2], target_pos[2])
                
                # If we're too low, prioritize ascending
                if current_pos[2] < self.min_altitude:
                    z_control = abs(z_control)  # Force positive
                
                # If we're descending near minimum altitude, limit descent rate
                if z_control < 0 and current_pos[2] < (self.min_altitude + 0.3):
                    z_control = max(z_control, -0.1)  # Slow descent
                
                commands[i, 2] = float(np.clip(
                    z_control,
                    -self.max_linear_velocity/2, self.max_linear_velocity/2
                ))
                
                # Yaw control (follow leader's yaw for followers)
                yaw_target = self.leader_yaw if i > 0 else 0.0
                cyaw[i] = float(np.clip(
                    self.pids[name]['yaw'].update(euler_c[2], yaw_target),
                    -self.max_angular_velocity, self.max_angular_velocity
                ))
                    
            # Publish commands
            for i, name in enumerate(self.robot_names):
                if self.robot_poses[name] is None:
                    continue
                    
                cmd = Twist()
                cmd.linear.x = float(commands[i, 0])
                cmd.linear.y = float(commands[i, 1])
                cmd.linear.z = float(commands[i, 2])
                cmd.angular.z = float(cyaw[i])
                self.cmd_pubs[name].publish(cmd)

        except Exception as e:
            self.get_logger().error(f"Error in automatic control: {str(e)}")
            self.enable = False
            self.publish_enable_state()

def main(args=None):
    rclpy.init(args=args)
    controller = FormationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()