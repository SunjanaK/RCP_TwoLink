# clock_pid.py
#
# Sample Webots controller file for driving the clock model
# with two independent driven joints using the
# Webots PID control model.

# No copyright, 2020, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("loading clock_pid.py...")

# Import the Webots simulator API.
from controller import Robot
import math, time

# Define the time step in milliseconds between controller updates.
EVENT_LOOP_DT = 1000

# Request a proxy object representing the robot to control.
robot = Robot()
robot_name = robot.getName()
print("%s: controller connected." % (robot_name))

# Fetch handle for the minute and hour hand motors.
minute = robot.getDevice('minuteMotor')
hour   = robot.getDevice('hourMotor')

# Adjust the low-level controller gains.
print("%s: setting PID gains." % (robot_name))
minute.setControlPID( 4.0, 0.0, 0.0)
hour.setControlPID(   4.0, 0.0, 0.0)

# Run loop to execute a periodic script until the simulation quits.
# If the controller returns -1, the simulator is quitting.
while robot.step(EVENT_LOOP_DT) != -1:

    # Fetch the current wall clock time.
    now = time.localtime(time.time())

    # Convert to motor angles in radians.
    hour_angle   = (now.tm_hour % 12) * (math.pi/6)
    minute_angle = (now.tm_min % 60) * (math.pi/30)

    # Update the position targets.
    hour.setPosition(hour_angle)
    minute.setPosition(minute_angle)
