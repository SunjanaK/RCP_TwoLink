# painting.py
#
# Sample Webots controller file for a 'robot' simulating
# a paintable surface using a Display node. The object
# received painting over the simulated radio network.

# No copyright, 2020-2021, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("loading painting.py...")

################################################################
# standard Python libraries
import math

# Import the Webots simulator API.
from controller import Robot

################################################################
# Define the time step in milliseconds between controller updates.
EVENT_LOOP_DT = 100

################################################################
class Painting(Robot):

    def __init__(self):
        super(Painting, self).__init__()

        self.robot_name = self.getName()
        print("%s: controller connected." % (self.robot_name))

        # Connect to the radio receiver.  The radio runs at the same rate as the
        # event loop as it is the main function of this robot.
        self.receiver = self.getDevice('receiver')
        self.receiver.setChannel(1)
        self.receiver.enable(EVENT_LOOP_DT)

        # Connect to the display object.
        self.display = self.getDevice('painting')
        self.width = self.display.getWidth()
        self.height = self.display.getHeight()
        print("%s: display is %d x %d pixels" % (self.robot_name, self.width, self.height))
        self.display.setAlpha(1.0)
        self.display.setOpacity(1.0)
        self.display.setColor(0xff0000)
        return

    #================================================================
    def poll_receiver(self):
        """Process all available radio messages, interpreting them as drawing commands."""
        while self.receiver.getQueueLength() > 0:
            packet = self.receiver.getData()
            tokens = packet.split()
            if len(tokens) < 2:
                print("%s malformed packet: %s" % (self.robot_name, packet))
            else:
                name = tokens[0].decode() # convert bytestring to Unicode
                # print("%s painter data: %s" % (self.robot_name, tokens))
                try:
                    bx = float(tokens[1])
                    by = float(tokens[2])
                    # scale body XY to display pixels
                    x = int((bx + 1.0)*128)
                    y = int((1.0 - by)*128)
                    self.display.fillOval(x, y, 5, 5)
                    print("%s painting at %d, %d" % (self.robot_name, x, y))

                except ValueError:
                    print("%s malformed packet: %s", (self.robot_name, packet))

            # done with packet processing, advance to the next packet
            self.receiver.nextPacket()
        # no more data
        return

    #================================================================
    def run(self):

        # Run loop to execute a periodic script until the simulation quits.
        # If the controller returns -1, the simulator is quitting.
        while self.step(EVENT_LOOP_DT) != -1:

            # Poll the simulated radio receiver.
            self.poll_receiver()

################################################################

controller = Painting()
controller.run()
