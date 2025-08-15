import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp 
        self.Ki = Ki 
        self.Kd = Kd 
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, process_variable, dt):
        error = self.setpoint - process_variable

        p_out = self.Kp * error

        self.integral += error * dt
        i_out = self.Ki * self.integral

        derivative = (error - self.prev_error) / dt 
        d_out = self.Kd * derivative

        controller_output = p_out + i_out + d_out

        self.prev_error = error

        return controller_output

def main():
    setpoint = 100  # target speed
    pid = PIDController(Kp = 1.5, Ki = 0.2, Kd = 0.2, setpoint= setpoint)

    time = np.linspace(0, 20, 200)  # 10 seconds divided to 10 millisecond intervals
    dt = time[1] - time[0]
    process_variable = 20   # initial speed
    process_values = []

    for t in time:
        control_output = pid.compute(process_variable, dt)

        acceleration = control_output - 0.1 * process_variable  # 0.1 simulates drag
        process_variable += acceleration * dt

        process_values.append(process_variable) 
    
    plt.figure(figsize= (10, 6))
    plt.plot(time, process_values, label = 'Process Variable (Speed)')
    plt.axhline(y= setpoint, color= 'r', linestyle= '--', label= 'setpoint')
    plt.xlabel('Time')
    plt.ylabel('Speed')
    plt.legend()
    plt.grid()
    plt.show()

main()