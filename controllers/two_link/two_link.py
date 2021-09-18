# two_link.py
#
# Sample Webots controller file for driving the two-link arm
# with two driven joints.  This example simulates a passive
# distal link by applying zero torque, then moves the
# base joint in a periodic excitation.

# No copyright, 2020-2021, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("loading two_link.py...")

# Import the Webots simulator API.
from controller import Robot

# Define the time step in milliseconds between controller updates.
EVENT_LOOP_DT = 200

# Request a proxy object representing the robot to control.
robot = Robot()
robot_name = robot.getName()
print("%s: controller connected." % (robot_name))

# Fetch handle for the 'base' and 'elbow' joint motors.
j1 = robot.getDevice('motor1')
j2 = robot.getDevice('motor2')

# Configure the motor for velocity control by setting
# the position targets to infinity.
j1.setPosition(float('inf'))
j2.setPosition(float('inf'))

# Start out with a 3 radian/second target rotational
# velocity (roughly 180 deg/sec).
j1.setVelocity(0)
j2.setVelocity(0)

# Configure the second motor to freewheel.  Please note
# this does not turn off the hinge friction.  For reference see:
#  https://cyberbotics.com/doc/reference/motor
#  https://cyberbotics.com/doc/reference/rotationalmotor
# j2.setTorque(0.0)

# Run loop to execute a periodic script until the simulation quits.
# If the controller returns -1, the simulator is quitting.
while robot.step(EVENT_LOOP_DT) != -1:
    # Read simulator clock time.
    t = robot.getTime()
    # Change the target velocity in a cycle with a two-second period.
    if int(t) % 10 < 5:
        j1.setVelocity(-.75)
        j2.setVelocity(.25)
    else:
        j1.setVelocity(.75)
        j2.setVelocity(-.5)
        
        
        
