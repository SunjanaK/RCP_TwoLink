# clock.py

# Sample Webots controller file for driving the clock
# model using torque mode and an implementation of a
# linear PD controller on the two independent driven
# joints.

# No copyright, 2020, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("loading clock.py...")

# Import the Webots simulator API.
from controller import Robot
import math, time

# Define the controller update time step in milliseconds.
EVENT_LOOP_DT = 20

# Specify proportional and derivative (damping) gains.
P_gain = 0.1    # units are N-m / radian
D_gain = 0.07    # units are N-m / (radian/sec)

# Specify a soft torque limit.  The underlying actuator
# model is limited to a modest 0.1 N-m; this limits the
# request to avoid error messages.
max_tau = 0.1

# Request a proxy object representing the robot to control.
robot = Robot()
robot_name = robot.getName()
print("%s: controller connected." % (robot_name))

# Fetch handles for the minute and hour hand motors.
minute_motor = robot.getDevice('minuteMotor')
hour_motor   = robot.getDevice('hourMotor')

# Enter torque motor on both motors; this bypasses
# the Webots low-level PID controllers.
minute_motor.setTorque(0)
hour_motor.setTorque(0)

# Fetch handles for the minute and hour hand sensors.
minute_sensor = robot.getDevice('minuteSensor')
hour_sensor   = robot.getDevice('hourSensor')

# Specify the sampling rate for the joint sensors.
minute_sensor.enable(EVENT_LOOP_DT)
hour_sensor.enable(EVENT_LOOP_DT)

# The controller estimates velocities using finite
# differencing of the position sensors; these variables
# hold the previous state.
last_minute_angle = 0
last_hour_angle   = 0

# Fetch the current wall clock time.
now = time.localtime(time.time())

# Convert time to initial hand motor angles in radians.
initial_minute_angle = (now.tm_min  % 60) * (math.pi/30)
initial_hour_angle   = (now.tm_hour % 12) * (math.pi/6)

# Generate some debugging output
print("%s: initial joint angles: minute: %f, hour: %f" % (robot_name, initial_minute_angle, initial_hour_angle))
debug_timer = 2.0

# Run loop to execute a periodic script until the simulation quits.
# If the controller returns -1, the simulator is quitting.
while robot.step(EVENT_LOOP_DT) != -1:

    # Read simulator clock time and calculate new position targets based on elapsed time.
    sim_t = robot.getTime()
    target_minute = initial_minute_angle + sim_t * (2*math.pi / 3600)  # one rev  per  3600 seconds (hour)
    target_hour   = initial_hour_angle   + sim_t * (4*math.pi / 86400) # two revs per 86400 seconds (day)

    # Read the current sensor positions.
    minute_angle = minute_sensor.getValue()
    hour_angle   = hour_sensor.getValue()

    # Estimate current velocities in radians/sec using finite differences.
    d_minute_dt = (minute_angle - last_minute_angle) / (0.001 * EVENT_LOOP_DT)
    d_hour_dt   = (hour_angle   - last_hour_angle)   / (0.001 * EVENT_LOOP_DT)
    last_minute_angle = minute_angle
    last_hour_angle   = hour_angle

    # Calculate new motor torques, limit them, and apply them to the system.
    tau_minute = P_gain * (target_minute - minute_angle) - D_gain * d_minute_dt
    tau_hour   = P_gain * (target_hour   - hour_angle)   - D_gain * d_hour_dt

    tau_minute = min(max(tau_minute, -max_tau), max_tau)
    tau_hour   = min(max(tau_hour,   -max_tau), max_tau)

    minute_motor.setTorque(tau_minute)
    hour_motor.setTorque(tau_hour)

    # Occasionally issue a message for debugging.
    debug_timer -= 0.001*EVENT_LOOP_DT
    if debug_timer < 0.0:
        debug_timer += 2.0
        print("%s: motor torques: minute: %f, hour: %f" % (robot_name, tau_minute, tau_hour))
