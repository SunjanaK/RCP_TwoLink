# pendulum_1_2.py
#
# Sample Webots controller file for driving the
# underactuated 1-2 pendulum.  The robot has a driven
# base joint between the base and link1, and passive
# joints between link1 and the two distal links link2A
# and link2B.
#
# No copyright, 2020-2021, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("loading pendulum_1_2.py...")

# Import the Webots simulator API.
from controller import Robot

# Import the standard Python math library.
import math

print("pendulum_1_2.py waking up...")

# Define the time step in milliseconds between
# controller updates.
EVENT_LOOP_DT = 20

# Request a proxy object representing the robot to
# control.
robot = Robot()

# Fetch handles for the joint sensors.
j1  = robot.getDevice('joint1')
j2A = robot.getDevice('joint2A')
j2B = robot.getDevice('joint2B')

# Specify the sampling rate for the joint sensors.
j1.enable(EVENT_LOOP_DT)
j2A.enable(EVENT_LOOP_DT)
j2B.enable(EVENT_LOOP_DT)

# Fetch handle for the 'base' joint motor.  In this
# example the motor will be controlled as a torque
# motor, bypassing the lower-level PID control.
motor1 = robot.getDevice('motor1')
motor1.setTorque(0.0)

# Run an event loop until the simulation quits,
# indicated by the step function returning -1.
while robot.step(EVENT_LOOP_DT) != -1:

    # Read simulator clock time.
    t = robot.getTime()

    # Read the new joint positions.
    q1  = j1.getValue()
    q2A = j2A.getValue()
    q2B = j2B.getValue()

    # Compute and apply new base joint actuator torque.
    # In this example, the excitation is only based on
    # time, but could also be a function of the joint
    # positions.
    tau = 3 * math.sin(3*t)
    motor1.setTorque(tau)
