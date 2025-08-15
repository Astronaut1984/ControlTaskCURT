import numpy as np

class Vechicle:

    def __init__(self):
        # Vehicle parameters based on https://www.mathworks.com/help/vdynblks/ref/formulastudentvehicle.html 
        self.wheel_radius = 0.24
        self.wheel_base = 1.22
        self.max_speed = 36 # rough estimate in m/s
        self.ref_speed = 30 # nominal forward speed

        self.pose = (0.0, 0.0, 0.0) # x, y, theta

    def set_pose(self, x, y, theta):
        self.pose = (x, y, theta)

    def get_pose(self):
        return self.pose

    def forward_kinematics(self, v_right, v_left):

        v = (v_right + v_left) / 2
        omega = np.abs(v_right - v_left) / self.wheel_base

        return v, omega 

    def update_pose(self, v_left, v_right, dt):

        # Clamp the velocities, ensuring they stay between max_speed and -(max_speed)
        v_left = max(min(v_left, self.max_speed), -self.max_speed)
        v_right = max(min(v_right, self.max_speed), -self.max_speed)

        v, omega = self.forward_kinematics(v_right, v_left)

        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

        self.theta = max(min(self.theta, np.pi), -np.pi)

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki 
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
    
    def compute(self, setpoint, measured, dt):
        error = setpoint - measured
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class VehicleController:
    def __init__(self, kinematics: Vechicle, pid_linear:PIDController, pid_angular:PIDController):
        self.kinematics = kinematics
        self.pid_linear = pid_linear
        self.pid_angular = pid_angular

    def compute_wheel_velocities(self, target_pose, dt):
        curr_pose = self.kinematics.get_pose()

        dx = target_pose[0] - curr_pose[0]
        dy = target_pose[1] - curr_pose[1]
        
        distance_error = np.hypot(dx, dy)
        target_angle = np.arctan2(dx, dy)
        heading_error = (target_angle - curr_pose[2] + np.pi) % (2 * np.pi) - np.pi

        v = self.pid_linear.compute(0.0, -distance_error, dt)
        omega = self.pid_angular.compute(0.0, -heading_error, dt)

        r = self.kinematics.wheel_radius
        L = self.kinematics.wheel_base

        v_left = (2 * v + omega * L) / (2 * r)
        v_right = (2 * v - omega * L) / (2 * r)

        return v_left, v_right
    
    def update(self, target_pos, dt):
        v_left,v_right = self.compute_wheel_velocities(target_pose=target_pos, dt= dt)
        self.kinematics.update_pose(v_left, v_right, dt)


        
