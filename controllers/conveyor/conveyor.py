# conveyor.py
#
# Sample Webots controller file for driving the three-wheel
# conveyor demo.  The robot comprises three driven wheels
# arranged in a triangle.  A large passive object (the
# 'manipulandum') is balanced on top of the wheels for them to
# move.
#
# No copyright, 2021, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("loading conveyor.py...")

# Import the Webots simulator API.
from controller import Robot

# Import standard Python modules
import math, random

print("conveyor.py waking up...")

# Define the time step in milliseconds between
# controller updates.
EVENT_LOOP_DT = 20

# Request a proxy object representing the robot to
# control.
robot = Robot()

# Fetch handles for the joint sensors.
sensors = (robot.getDevice('joint1'),
           robot.getDevice('joint2'),
           robot.getDevice('joint3'))

# Specify the sampling rate for the joint sensors.
for s in sensors:
    s.enable(EVENT_LOOP_DT)

# Fetch handle for each conveyor wheel motor.  The motor1 axis
# is positioned along X, so rotations will move the object along
# Y.  Motor2 is 120 degrees CCW from motor1.
motors = (robot.getDevice('motor1'),
          robot.getDevice('motor2'),
          robot.getDevice('motor3'))

# Define a few constant which match the model to be used for
# kinematics calculations.  All units are meters to match the
# simulation.
base_radius  = 0.3
wheel_radius = 0.05

wheel_origins = ((0.3, 0.0, 0.1),
                 (-0.15,0.259808, 0.1),
                 (-0.15,-0.259808,0.1))

wheel_axes    = ((1.0,0.0,0.0),
                 (-0.5,0.866025,0),
                 (-0.5,-0.866025,0))

# The wheel tangent axes define the direction of travel of the
# top contact point for a positive wheel rotation.
wheel_tangents = ((0.0,-1.0,0.0),
                  (0.866025, 0.5,0),
                  (-0.866025, 0.5,0))

# In this example the motor will use velocity control,
# configured by setting an infinite position target and then
# using setVelocity() to issue motor commands.
for motor in motors:
    motor.setPosition(math.inf)
    motor.setVelocity(0)

################################################################
# Dot product utility function.  It would be better to use
# numpy, but this avoids adding the dependency.
def dot_product(v1, v2):
    accum = 0.0
    for e1, e2 in zip(v1, v2):
        accum += e1*e2
    return accum

################################################################
# Define action primitives to set the motor velocities for
# particular manipulandum motions.

def stop_motion(motors):
    for m in motors:
        m.setVelocity(0)

def random_motion(motors):
    for m in motors:
        m.setVelocity(4*random.random() - 2.0)

# Spin the object around the base center at the angular velocity
# specified in radians/sec.  N.B. 180 degrees == pi radians, 1
# radian is about 57 degrees.
def spin_around_center(motors, angvel=math.pi/12):
    motor_vel = -(angvel*base_radius)/wheel_radius
    print(f"All motors set to {motor_vel} rad/sec.")
    for m in motors:
        m.setVelocity(motor_vel)

# Move the object along a XYZ velocity vector expressed in
# meters/sec.  This take the dot product of the velocity vector
# with each wheel tangent vector to find the contribution of
# each wheel to the linear velocity, then scales by the wheel
# radius to each wheel rotational velocity.
def move_along_vector(motors, velvec):
    qd = [dot_product(velvec, tangent)/wheel_radius for tangent in wheel_tangents]
    for m, v in zip(motors, qd):
        m.setVelocity(v)
    print(f"Applying motor velocities {qd} rad/sec.")

# Move the object along the X axis at the linear velocity specified in
# meters/second.
def move_along_x(motors, vel=0.1):
    # Take the dot product of the velocity vector with each
    # wheel tangent vector to find the contribution of each
    # wheel to the linear velocity, then scale by the wheel
    # radius to find the rotational velocity.
    move_along_vector(motors, (vel, 0.0, 0.0))

# Move the object along the Y axis at the linear velocity specified in
# meters/second.
def move_along_y(motors, vel=0.1):
    # Take the dot product of the velocity vector with each
    # wheel tangent vector to find the contribution of each
    # wheel to the linear velocity, then scale by the wheel
    # radius to find the rotational velocity.
    move_along_vector(motors, (0.0, vel, 0.0))

    
################################################################
# Define a few global variables for the behavior state machine.
# This would be better represented as an object class.
state_index = 'start'
state_timer = 0
state_new_flag = False

# Utility function for state machine transitions.
def transition(next_state):
    global state_timer, state_index, state_new_flag
    state_index = next_state
    state_timer = 0.0
    state_new_flag = True
    print(f"Entering state {next_state}.")

################################################################    
# Run an event loop until the simulation quits,
# indicated by the step function returning -1.
while robot.step(EVENT_LOOP_DT) != -1:

    # Read simulator clock time.
    t = robot.getTime()

    # Read the new joint positions.
    q = [joint.getValue() for joint in sensors]

    # Evaluate a state machine to switch between action primitives.
    state_timer -= 0.001 * EVENT_LOOP_DT
    state_is_new = state_new_flag
    state_new_flag = False
    
    if state_index == 'start':
        transition('spinning1')

    elif state_index == 'spinning1':
        if state_is_new:
            state_timer = 2.0
            spin_around_center(motors, 0.2)
        elif state_timer < 0.0:
            transition('pause1')
            
    elif state_index == 'pause1':
        if state_is_new:
            state_timer = 1.0
            stop_motion(motors)
        elif state_timer < 0.0:
            transition('X+')

    elif state_index == 'X+':
        if state_is_new:
            move_along_x(motors, 0.1)
            state_timer = 0.5
        elif state_timer < 0.0:
            transition('X-')

    elif state_index == 'X-':
        if state_is_new:
            move_along_x(motors, -0.1)
            state_timer = 1.5
        elif state_timer < 0.0:
            transition('pause2')

    elif state_index == 'pause2':
        if state_is_new:
            state_timer = 1.0
            stop_motion(motors)
        elif state_timer < 0.0:
            transition('spinning2')

    elif state_index == 'spinning2':
        if state_is_new:
            spin_around_center(motors, -0.2)
            state_timer = 1.0
        elif state_timer < 0.0:
            transition('Y+')

    elif state_index == 'Y+':
        if state_is_new:            
            move_along_y(motors, 0.1)
            state_timer = 0.5
        elif state_timer < 0.0:
            transition('Y-')

    elif state_index == 'Y-':
        if state_is_new:                    
            move_along_y(motors, -0.2)
            state_timer = 1.5
        elif state_timer < 0.0:
            transition('pause3')

    elif state_index == 'pause3':
        if state_is_new:
            state_timer = 1.0
            stop_motion(motors)
        elif state_timer < 0.0:
            transition('spinning1')
