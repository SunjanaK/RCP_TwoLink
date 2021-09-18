# two_link_mqtt.py
#
# Sample Webots controller file for driving the
# two-link arm with two driven joints.  This example
# simulates a passive distal link by applying zero
# torque, then moves the base joint in a periodic
# excitation.  It also opens a connection to an MQTT
# server to communicate with a remote system across the
# network.

# No copyright, 2020-2021, Garth Zeglin.  This file is
# explicitly placed in the public domain.

################################################################
# standard Python libraries
import getpass

# Import the Webots simulator API.
from controller import Robot

# Import the Paho MQTT client library.
# documentation: https://www.eclipse.org/paho/clients/python/docs/
import paho.mqtt.client as mqtt

################################################################
print("two_link_mqtt.py waking up.")

HOST      = "mqtt.ideate.cmu.edu"  # IDeATe MQTT server hostname
PORT      = 8885                   # port of server instance for 16-375
USER      = "students"             # Specific login for this server.
PASSWORD  = "<password>"           # Specific password for this login.

# Create a default subscription and and default
# broadcast topic based on the current username.
# Please customize this as needed.  Subscriptions can
# also include a path with # as a wildcard,
# e.g. 'username/robot1/#'.
username = getpass.getuser()
subscription = username + '/input'   # message topic(s) to receive
send_topic   = username + '/output'  # message topic to send

# Declare some MQTT callbacks.
def on_connect(client, userdata, flags, rc):
    # Subscribing in on_connect() means that if we lose the connection and reconnect then subscriptions will be renewed.    
    global mqtt_client    
    print("MQTT connected to server with with flags: %s, result code: %s" % (flags, rc))
    print("MQTT subscribing to", subscription)
    mqtt_client.subscribe(subscription)
    
def on_message(client, userdata, msg):
    print("MQTT message received: {%s} %s" % (msg.topic, msg.payload))

# Initialize the MQTT client system
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.tls_set()
mqtt_client.username_pw_set(USER, PASSWORD)

# Define the time step in milliseconds between controller updates.
EVENT_LOOP_DT = 200

# Request a proxy object representing the robot to control.
robot = Robot()
robot_name = robot.getName()
print("%s: controller connected." % (robot_name))

# Connect to the MQTT network
print("Initiating MQTT connection to %s:%d" % (HOST, PORT))
mqtt_client.connect(host=HOST, port=PORT)

# Fetch handle for the 'base' and 'elbow' joint motors.
motor1 = robot.getDevice('motor1')
motor2 = robot.getDevice('motor2')

# Configure the motor for velocity control by setting
# the position targets to infinity.
motor1.setPosition(float('inf'))

# Start out with a 3 radian/second target rotational
# velocity (roughly 180 deg/sec).
motor1.setVelocity(3)

# Configure the second motor to freewheel.  Please note
# this does not turn off the hinge friction.  For reference see:
#  https://cyberbotics.com/doc/reference/motor
#  https://cyberbotics.com/doc/reference/rotationalmotor
motor2.setTorque(0.0)

# Fetch handles for the joint sensors.
joint1 = robot.getDevice('joint1')
joint2 = robot.getDevice('joint2')

# Specify the sampling rate for the joint sensors.
joint1.enable(EVENT_LOOP_DT)
joint2.enable(EVENT_LOOP_DT)

# Connect to the end sensor.
sensor = robot.getDevice("endRangeSensor")
sensor.enable(EVENT_LOOP_DT) # set sampling period in milliseconds

# Timer variables to control issue of status updates to the network.
message_output_interval = 1000  # milliseconds
message_output_timer    = 0

# Run loop to execute a periodic script until the simulation quits.
# If the controller returns -1, the simulator is quitting.
while robot.step(EVENT_LOOP_DT) != -1:

    # Poll the network system.
    mqtt_client.loop(timeout=0.0)

    # Read simulator clock time.
    t = robot.getTime()

    # Change the target velocity in a cycle with a two-second period.
    if int(t) % 2 == 0:
        motor1.setVelocity(0)
    else:
        motor1.setVelocity(3)

    # Update the network as needed.
    message_output_timer -= EVENT_LOOP_DT
    if message_output_timer < 0:
        message_output_timer += message_output_interval
        mqtt_client.publish(send_topic, "%f %f %f %f" %(t, joint1.getValue(), joint2.getValue(), sensor.getValue()))
        
# Shut down the networking cleanly.  At this point
# print() output will no longer show in the Webots
# console, so this is silent.
mqtt_client.disconnect()
