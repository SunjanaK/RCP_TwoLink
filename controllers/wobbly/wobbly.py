# wobbly.py

# Sample Webots controller file for driving the wobbly diff-drive
# mobile robot.

# No copyright, 2020-2021, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("loading wobbly.py...")

# Import the Webots simulator API.
from controller import Robot

# Import standard Python libraries.
import math, random, time

# Define the time step in milliseconds between controller updates.
EVENT_LOOP_DT = 200

################################################################
class Wobbly(Robot):
    def __init__(self):

        super(Wobbly, self).__init__()
        self.robot_name = self.getName()
        print("%s: controller connected." % (self.robot_name))

        # Attempt to randomize the random library sequence.
        random.seed(time.time())

        # Initialize geometric constants.  These should match
        # the current geometry of the robot.
        self.wheel_radius = 0.1
        self.axle_length  = 0.14

        # Fetch handles for the wheel motors
        self.l_motor = self.getDevice('left wheel motor')
        self.r_motor = self.getDevice('right wheel motor')

        # Adjust the low-level controller gains.
        print("%s: setting PID gains." % (self.robot_name))
        self.l_motor.setControlPID(1.0, 0.0, 0.1)
        self.r_motor.setControlPID(1.0, 0.0, 0.1)

        # Fetch handles for the wheel joint sensors.
        self.l_pos_sensor = self.getDevice('left wheel sensor')
        self.r_pos_sensor = self.getDevice('right wheel sensor')

        # Specify the sampling rate for the joint sensors.
        self.l_pos_sensor.enable(EVENT_LOOP_DT)
        self.r_pos_sensor.enable(EVENT_LOOP_DT)

        # Connect to the eye sensors.
        self.l_eye_sensor = self.getDevice('leftDistanceSensor')
        self.r_eye_sensor = self.getDevice('rightDistanceSensor')
        self.l_eye_sensor.enable(EVENT_LOOP_DT)
        self.r_eye_sensor.enable(EVENT_LOOP_DT)

        # Connect to the radio emitter and receiver.
        self.receiver = self.getDevice('receiver')
        self.emitter  = self.getDevice('emitter')
        self.radio_interval = 1000
        self.radio_timer = 0
        self.receiver.enable(self.radio_interval)

        # Maintain a table of peer robot locations received over the radio.
        self.peers = {}

        # Connect to the GPS position sensor.
        self.gps = self.getDevice('gps')
        self.gps_timer = 0
        self.gps_interval = 1000
        self.gps.enable(self.gps_interval)
        self.gps_location = [0, 0, 0.1] # reference pose value

        # Connect to the compass orientation sensor.
        self.compass = self.getDevice('compass')
        self.compass_timer = 0
        self.compass_interval = 200
        self.compass.enable(self.compass_interval)

        # State variables for reporting compass readings.  The heading is the
        # body forward (body +X) direction expressed in degrees clockwise from
        # North (world +Y).  The compass_vector is the direction of North
        # expressed in body coordinates.  The neutral pose points East.
        self.heading = 90
        self.compass_vector = [0,1] # reference pose value
        self.heading_error = 0

        # Connect to the accelerometer sensor.
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer_timer = 0
        self.accelerometer_interval = 200
        self.accelerometer.enable(self.accelerometer_interval)
        self.accel_vector = [0, 0, 9.81] # reference pose value

        # Initialize generic behavior state machine variables.
        self.state_timer = 0        # timers in milliseconds
        self.state_index = 0        # current state
        self.target_heading = 0
        self.target_velocity = 0
        return

    #================================================================
    # Polling function to process sensor input at different timescales.
    def poll_sensors(self):

        self.gps_timer -= EVENT_LOOP_DT
        if self.gps_timer < 0:
            self.gps_timer += self.gps_interval
            location = self.gps.getValues()
            if not math.isnan(location[0]):
                self.gps_location = location
                # print("%s GPS: %s" % (self.robot_name, location))

        self.compass_timer -= EVENT_LOOP_DT
        if self.compass_timer < 0:
            self.compass_timer += self.compass_interval
            orientation = self.compass.getValues()
            if not math.isnan(orientation[0]) and not math.isnan(orientation[1]):
                # For heading, 0 degrees is North, 90 is East, 180 is South, 270 is West.
                # The world is assumed configured 'ENU' so X is East and Y is North.
                # The robot 'front' is along body +X, so the neutral pose is facing East.
                self.heading = math.fmod(2*math.pi + math.atan2(orientation[1], orientation[0]), 2*math.pi) * (180.0/math.pi)
                self.compass_vector = orientation[0:2]
                # print("%s Compass: %s, heading %3.0f deg" % (self.robot_name, self.compass_vector, heading))

        self.accelerometer_timer -= EVENT_LOOP_DT
        if self.accelerometer_timer < 0:
            self.accelerometer_timer += self.accelerometer_interval
            self.accel_vector = self.accelerometer.getValues()
            # The accelerometer will read [0, 0, 9.81] when stationary in the reference pose.
            # print("%s Accelerometer: %s" % (self.robot_name, self.accel_vector))

        return

    #================================================================
    # Polling function to process radio and network input at different timescales.
    def poll_communication(self):
        self.radio_timer -= EVENT_LOOP_DT
        if self.radio_timer < 0:
            self.radio_timer += self.radio_interval
            while self.receiver.getQueueLength() > 0:
                packet = self.receiver.getData()
                # print("%s Receiver: %s" % (self.robot_name, packet))
                tokens = packet.split()
                if len(tokens) != 5:
                    print("%s malformed packet: %s" % (self.robot_name, packet))
                else:
                    name = tokens[0].decode() # convert bytestring to Unicode
                    if self.peers.get(name) is None:
                        print("%s receiver: new peer observed: %s" % (self.robot_name, name))

                    self.peers[name] = {'location' : [float(tokens[1]), float(tokens[2]), float(tokens[3])],
                                        'heading'  : float(tokens[4]),
                                        'timestamp' : self.getTime(),
                                        }

                # done with packet processing
                self.receiver.nextPacket()

            # Transmit a status message at the same rate
            name_token = self.robot_name.replace(" ","_")
            status = "%s %.2f %.2f %.2f %.0f" % (name_token, self.gps_location[0], self.gps_location[1],
                                                 self.gps_location[2] - self.wheel_radius, self.heading)

            # emitter requires a bytestring, not a Python Unicode string
            data = status.encode()

            # print("%s emitter: sending %s" % (self.robot_name, data))
            self.emitter.send(data)

    #================================================================
    # motion primitives
    def go_forward(self, velocity):
        """Command the motor to turn at the rate which produce the ground velocity
           specified in meters/sec.  Negative values turn backward. """

        # velocity control mode
        self.l_motor.setPosition(math.inf)
        self.r_motor.setPosition(math.inf)

        # calculate the rotational rate in radians/sec based on the wheel radius
        theta_dot = velocity / self.wheel_radius
        self.l_motor.setVelocity(theta_dot)
        self.r_motor.setVelocity(theta_dot)
        return

    def go_rotate(self, rot_velocity):
        """Command the motors to turn in place at the rate which produce the rotational
           velocity specified in radians/sec.  Negative values turn
           backward."""

        # velocity control mode
        self.l_motor.setPosition(math.inf)
        self.r_motor.setPosition(math.inf)

        # calculate the difference in linear velocity of the wheels
        linear_velocity = self.axle_length * rot_velocity

        # calculate the net rotational rate in radians/sec based on the wheel radius
        theta_dot = linear_velocity / self.wheel_radius

        # apply the result symmetrically to the wheels
        self.l_motor.setVelocity( 0.5*theta_dot)
        self.r_motor.setVelocity(-0.5*theta_dot)
        return

    def heading_difference(self, target, current):
        """Calculate a directional error in degrees, always returning a value on (-180, 180]."""
        err = target - current
        # fold the range of values to (-180, 180]
        if err > 180.0:
            return err - 360
        elif err <= -180.0:
            return err + 360
        else:
            return err

    def go_heading(self, target_heading):
        """Rotate toward the heading specifed in positive degrees, with 0 at North (+Y),
           90 at East (+X).  This assume the compass-reading process is
           active."""

        # find the directional error in degrees
        self.heading_error = self.heading_difference(target_heading, self.heading)

        # apply a linear mapping from degrees error to rotational velocity in radians/sec
        rot_vel = 0.02 * self.heading_error
        self.go_rotate(rot_vel)
        # print("go_heading: %f, %f, %f" % (target_heading, self.heading, rot_vel))
        return

    def go_still(self):
        """Actively damp any wobble to come to rest in place."""
        # map an error in X acceleration to a linear velocity
        vel = -0.05 * self.accel_vector[0]
        self.go_forward(vel)
        # print("go_still: %f, %f" % (self.accel_vector[0], vel))
        return

    #================================================================
    def peer_heading_distance(self, record):
        """Given a peer record, return a tuple (heading, distance) with the compass
           heading and distance in meters of the peer from this robot current
           location."""
        loc = record['location']
        dx = loc[0] - self.gps_location[0]
        dy = loc[1] - self.gps_location[1]
        distance = math.sqrt(dx*dx + dy*dy)
        heading  = math.fmod(2*math.pi + math.atan2(dx, dy), 2*math.pi) * (180.0/math.pi)
        return heading, distance

    def nearest_peer(self, range=2.0):
        """Locate the nearest peer (as reported by radio) within the given range.
           Returns either None or a dictionary with the location record."""
        result = None
        best = math.inf
        for name in self.peers:
            record = self.peers[name]
            heading, dist = self.peer_heading_distance(record)
            if dist < best:
                result = record
                best = dist
        return result

    #================================================================
    def poll_wandering_activity(self):
        """State machine update function to aimlessly wander around the world."""

        # This machine always transitions at regular intervals.
        timer_expired = False
        if self.state_timer < 0:
            self.state_timer += 3000
            timer_expired = True

        # Evaluate the side-effects and transition rules for each state.
        if self.state_index == 0:
            print("Init state, entering cycle.")
            self.state_index = 1

        elif self.state_index == 1:
            self.go_forward(0.2)

            if timer_expired:
                self.state_index += 1

        elif self.state_index == 2:
            self.go_heading(self.target_heading)

            if timer_expired:
                self.state_index += 1

        elif self.state_index == 3:
            self.go_still()

            if timer_expired:
                self.state_index += 1

        elif self.state_index == 4:
            self.go_rotate(math.pi / 6)

            if timer_expired:
                self.state_index = 1
                self.target_heading = random.randint(0,360)

        else:
            print("%s: invalid state, resetting." % (self.robot_name))
            self.state_index = 0

        if timer_expired:
            print("%s: transitioning to state %s" % (self.robot_name, self.state_index))

        return

    #================================================================
    def poll_following_activity(self):
        """State machine update function to always move toward the nearest peer."""

        if self.state_timer < 0:
            self.state_timer += 1000

            # periodically test if there is a nearby peer
            nearest = self.nearest_peer()
            if nearest is None:
                self.state_index = 1
            else:
                self.state_index = 2
                heading, distance = self.peer_heading_distance(nearest)
                self.target_heading = heading
                self.target_velocity = 0.1 * distance
                print("%s: peer to follow at %f deg, %f meters" % (self.robot_name, heading, distance))

        # always either stabilize, turn, or move
        if self.state_index < 1:
            self.go_still()

        else:
            heading_err = self.heading_difference(self.target_heading, self.heading)
            if abs(heading_err) > 20.0 or abs(self.target_velocity) < 0.05:
                self.go_heading(self.target_heading)
            else:
                self.go_forward(self.target_velocity)

    #================================================================
    def run(self):
        # Run loop to execute a periodic script until the simulation quits.
        # If the controller returns -1, the simulator is quitting.
        while self.step(EVENT_LOOP_DT) != -1:
            # Read simulator clock time.
            self.sim_time = self.getTime()

            # Read sensor values.
            self.poll_sensors()

            # Check the radio and/or network.
            self.poll_communication()

            # Update the activity state machine.
            self.state_timer -= EVENT_LOOP_DT

            # This will run some open-loop motion.  One robot will be the leader, the rest will follow.
            mode = self.getCustomData()
            if mode == 'leader':
                self.poll_wandering_activity()
            else:
                self.poll_following_activity()


################################################################
# Start the script.
robot = Wobbly()
robot.run()
