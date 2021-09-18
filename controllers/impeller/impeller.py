# impeller.py
#
# Sample Webots controller file for driving a single actuator
# attached to an 'impeller' which manipulates objects.  The
# example simply moves in a periodic oscillation.

# No copyright, 2020-2021, Garth Zeglin.  This file is
# explicitly placed in the public domain.

#------------------------------------------------------------------------------
print("loading impeller.py...")

# Import standard Python modules.
import math

# Import the Webots simulator API.
from controller import Robot

# Define the time step in milliseconds between controller updates.
EVENT_LOOP_DT = 50

#------------------------------------------------------------------------------
# Request a proxy object representing the robot to control.
robot = Robot()
robot_name = robot.getName()
print("%s: controller connected." % (robot_name))

# Fetch handle for the 'base' joint motors.
j1 = robot.getDevice('motor1')

# Connect to the proximity sensor.
sensor = robot.getDevice("endRangeSensor")
sensor.enable(EVENT_LOOP_DT) # set sampling period in milliseconds

#------------------------------------------------------------------------------
# Run loop to execute a periodic script until the simulation quits.
# If the controller returns -1, the simulator is quitting.
while robot.step(EVENT_LOOP_DT) != -1:
    # Read simulator clock time.
    t = robot.getTime()

    # Change the target position target (in radians).
    period  =  6   # seconds
    degrees = 180 * math.sin(t * (2*math.pi / period))
    radians = degrees * (math.pi/180.0)
    j1.setPosition(radians)
