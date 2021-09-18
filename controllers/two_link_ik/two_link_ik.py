# two_link_ik.py
#
# Sample Webots controller file for driving the two-link arm
# with two driven joints.  This example provides inverse kinematics for
# performing position-controlled trajectories.

# No copyright, 2020-2021, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("two_link_ik.py waking up.")

# Import the Webots simulator API.
from controller import Robot

# Import the standard Python math library.
import math, random, time

# Define the time step in milliseconds between controller updates.
EVENT_LOOP_DT = 100

################################################################
class TwoLink(Robot):
    def __init__(self):

        super(TwoLink, self).__init__()
        self.robot_name = self.getName()
        print("%s: controller connected." % (self.robot_name))

        # Attempt to randomize the random library sequence.
        random.seed(time.time())

        # Initialize geometric constants.  These should match
        # the current geometry of the robot.
        self.link1_length = 0.5
        self.link2_length = 0.5

        # Fetch handle for the 'base' and 'elbow' joint motors.
        self.motor1 = self.getDevice('motor1')
        self.motor2 = self.getDevice('motor2')

        # Adjust the low-level controller gains.
        print("%s: setting PID gains." % (self.robot_name))
        self.motor1.setControlPID(2.0, 0.0, 0.1)
        self.motor2.setControlPID(2.0, 0.0, 0.1)

        # Fetch handles for the joint sensors.
        self.joint1 = self.getDevice('joint1')
        self.joint2 = self.getDevice('joint2')

        # Specify the sampling rate for the joint sensors.
        self.joint1.enable(EVENT_LOOP_DT)
        self.joint2.enable(EVENT_LOOP_DT)

        # Connect to the end sensor.
        self.end_sensor = self.getDevice("endRangeSensor")
        self.end_sensor.enable(EVENT_LOOP_DT) # set sampling period in milliseconds
        self.end_sensor_interval = 1000
        self.end_sensor_timer = 1000

        # Connect to the radio emitter and receiver.
        self.receiver = self.getDevice('receiver')
        self.emitter  = self.getDevice('emitter')
        self.radio_interval = EVENT_LOOP_DT
        self.radio_timer = 0
        self.emitter.setChannel(1)
        self.receiver.setChannel(1)
        self.receiver.enable(self.radio_interval)

        # Initialize behavior state machines.
        self.state_timer = 2*EVENT_LOOP_DT
        self.state_index = 0
        return

    #================================================================
    def forward_kinematics(self, q):
        """Compute the forward kinematics.  Returns the body-coordinate XY Cartesian
        position of the elbow and endpoint for a given joint angle vector.

        :param q: two-element list with [q1, q2] joint angles
        :return: tuple (elbow, end) of two-element lists with [x,y] locations
        """

        elbow = [self.link1_length * math.cos(q[0]), self.link1_length * math.sin(q[0])]
        end   = [elbow[0] + self.link2_length * math.cos(q[0]+q[1]), elbow[1] + self.link2_length * math.sin(q[0]+q[1])]
        return elbow, end

    #================================================================
    def endpoint_inverse_kinematics(self, target):
        """Compute two inverse kinematics solutions for a target end position.  The
        target is a XY Cartesian position vector in body coordinates, and the
        result vectors are joint angles as lists [q0, q1].  If the target is out
        of reach, returns the closest pose.
        """

        # find the position of the point in polar coordinates
        radiussq = target[0]**2 + target[1]**2
        radius   = math.sqrt(radiussq)

        # theta is the angle of target point w.r.t. X axis, same origin as arm
        theta    = math.atan2(target[1], target[0])

        # use the law of cosines to compute the elbow angle
        #   R**2 = l1**2 + l2**2 - 2*l1*l2*cos(pi - elbow)
        #   both elbow and -elbow are valid solutions
        acosarg = (radiussq - self.link1_length**2 - self.link2_length**2) / (-2 * self.link1_length * self.link2_length)
        if acosarg < -1.0:  elbow_supplement = math.pi
        elif acosarg > 1.0: elbow_supplement = 0.0
        else:               elbow_supplement = math.acos(acosarg)

        # use the law of sines to find the angle at the bottom vertex of the triangle defined by the links
        #  radius / sin(elbow_supplement)  = l2 / sin(alpha)
        if radius > 0.0:
            alpha = math.asin(self.link2_length * math.sin(elbow_supplement) / radius)
        else:
            alpha = 0.0

        #  compute the two solutions with opposite elbow sign
        soln1 = [theta - alpha, math.pi - elbow_supplement]
        soln2 = [theta + alpha, elbow_supplement - math.pi]

        return soln1, soln2
    #================================================================
    # motion primitives

    def go_target(self, target):
        """Issue a position command to move to the given endpoint position."""
        p1, p2 = self.endpoint_inverse_kinematics(target)

        # arbitrarily pick the first solution
        self.motor1.setPosition(p1[0])
        self.motor2.setPosition(p1[1])
        print("%s: target (%f, %f), moving to (%f, %f)" % (self.robot_name, target[0], target[1], p1[0], p1[1]))

    #================================================================
    # Polling function to process sensor input at different timescales.
    def poll_sensors(self):
        self.end_sensor_timer -= EVENT_LOOP_DT
        if self.end_sensor_timer < 0:
            self.end_sensor_timer += self.end_sensor_interval

            # read the distance sensor
            distance = self.end_sensor.getValue()

            if distance < 0.9:
                print("%s: range sensor detected obstacle at %f." % (self.robot_name, distance))

    #================================================================
    # Polling function to process radio and network input at different timescales.
    def poll_communication(self):
        self.radio_timer -= EVENT_LOOP_DT
        if self.radio_timer < 0:
            self.radio_timer += self.radio_interval
            while self.receiver.getQueueLength() > 0:
                packet = self.receiver.getData()
                print("%s receiver: %s" % (self.robot_name, packet))

                # done with packet processing
                self.receiver.nextPacket()

            # Transmit a status message at the same rate
            name_token = self.robot_name.replace(" ","_")
            elbow, end = self.forward_kinematics([self.joint1.getValue(), self.joint2.getValue()])
            status = "%s %.2f %.2f" % (name_token, end[0], end[1])

            # emitter requires a bytestring, not a Python Unicode string
            data = status.encode()

            # print("%s emitter: sending %s" % (self.robot_name, data))
            self.emitter.send(data)

    #================================================================
    def poll_spirals_activity(self):
        """State machine update function to aimlessly spiral around."""

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
            self.motor1.setPosition(math.inf)
            self.motor1.setVelocity(0.0)
            self.motor2.setPosition(math.inf)
            self.motor2.setVelocity(0.3)
            if timer_expired:
                self.state_index += 1

        elif self.state_index == 2:
            self.motor1.setPosition(math.inf)
            self.motor1.setVelocity(0.3)
            self.motor2.setPosition(math.inf)
            self.motor2.setVelocity(0.0)
            if timer_expired:
                self.state_index = 1

    #================================================================
    def poll_box_activity(self):
        """State machine update function to walk the corners of a box."""

        # This machine always transitions at regular intervals.
        timer_expired = False
        if self.state_timer < 0:
            self.state_timer += 2000
            timer_expired = True

        # Evaluate the side-effects and transition rules for each state.
        if self.state_index == 0:
            print("Init state, entering cycle.")
            self.go_target([0.3, 0.0])
            self.state_index = 1

        elif self.state_index == 1:
            if timer_expired:
                self.go_target([0.6, 0.0])
                self.state_index += 1

        elif self.state_index == 2:
            if timer_expired:
                self.go_target([0.6, 0.3])
                self.state_index += 1

        elif self.state_index == 3:
            if timer_expired:
                self.go_target([0.3, 0.3])
                self.state_index += 1

        elif self.state_index == 4:
            if timer_expired:
                self.go_target([0.3, 0.0])
                self.state_index = 1

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
            # mode = self.getCustomData()
            # self.poll_spirals_activity()
            self.poll_box_activity()


################################################################
# Start the script.
robot = TwoLink()
robot.run()
