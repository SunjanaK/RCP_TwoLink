# disembod.py
#
# Sample Webots controller file for a disembodied
# supervisor robot which communicates between the
# local radio network and a remote MQTT server.

# No copyright, 2020-2021, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("loading disembod.py...")

################################################################
# standard Python libraries
import math, getpass

# Import the Paho MQTT client library.
# documentation: https://www.eclipse.org/paho/clients/python/docs/
import paho.mqtt.client as mqtt

# Import the Webots simulator API.
from controller import Supervisor

################################################################

HOST      = "mqtt.ideate.cmu.edu"  # IDeATe MQTT server hostname
PORT      = 8885                   # port of server instance for 16-375
USER      = "students"             # Specific login for this server.
PASSWORD  = "<password>"           # Specific password for this login.

# Define the time step in milliseconds between controller updates.
EVENT_LOOP_DT = 200

################################################################
class Disembod(Supervisor):

    def __init__(self):
        super(Disembod, self).__init__()

        self.robot_name = self.getName()
        print("%s: controller connected." % (self.robot_name))

        # Connect to the radio emitter and receiver.  The radio runs at the same
        # rate as the event loop as it is the main function of this robot.
        self.receiver = self.getDevice('receiver')
        self.emitter  = self.getDevice('emitter')
        self.receiver.enable(EVENT_LOOP_DT)

        # The custom data field is used to specify the name of the individual robot
        # whose data is broadcast over MQTT.
        self.public_robot = self.getCustomData().replace(" ","_")

        # Create a subscription to receive all party messages.  This will
        # include our own transmissions, but these will be filtered after receipt.
        self.subscription = 'party/#'                # message topics to receive

        # Create a default broadcast topic based on the current username.
        # Please customize this as needed.
        self.username = getpass.getuser()
        self.send_topic   = 'party/' + self.username # message topic to send

        # Initialize the MQTT client system
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.tls_set()
        self.mqtt_client.username_pw_set(USER, PASSWORD)

        # Initialize the list of available proxies.  Each entry is a tuple with
        # several field objects used to query and set its state.
        self.proxies = []

        # Initialize the mapping which allocates proxies to incoming robot names.
        self.proxy_for_name = {}

        # Locate proxy robots in the scene tree to use for displaying received data.
        self.find_proxies()
        print("%s: found %d proxies." % (self.robot_name, len(self.proxies)))

        return

    #================================================================
    def find_proxies(self):
        """Find the proxy robot bodies in the scene tree to use for displaying the
        position of remote robots.  This assumes the proxies were created using
        the proto file and have a base type of proxy rather than Robot, i.e.,
        were not expanded into base nodes."""

        # fetch the root node of the scene tree, a Group object containing all visible nodes
        root = self.getRoot()
        top_level = root.getField('children')
        num_items = top_level.getCount()

        # walk the list of top-level scene tree nodes to look for proxy objects
        for i in range(num_items):
            node = top_level.getMFNode(i)
            basetype = node.getBaseTypeName()
            typename = node.getTypeName()
            # print("Found base type", basetype)
            if basetype == 'Robot' and typename == 'proxy':
                name_field = node.getField('name')
                pos_field  = node.getField('translation')
                rot_field  = node.getField('rotation')
                self.proxies.append((node, name_field, pos_field, rot_field))
                name = name_field.getSFString()
                position = pos_field.getSFVec3f()
                # print("Found proxy %s at %s" % (name, position))

    def get_proxy(self, name):
        """Return a proxy body for the given robot name.  Either returns an existing
        proxy, allocates an available one, or returns None if no more are available."""
        proxy = self.proxy_for_name.get(name)
        if proxy is None and len(self.proxies) > 0:
            proxy = self.proxies[-1]
            del(self.proxies[-1])
            name_field = proxy[1]
            name_field.setSFString(name)
            self.proxy_for_name[name] = proxy
            print("%s: allocated proxy for %s" % (self.robot_name, name))
        return proxy

    def set_proxy_pose(self, proxy, x, y, z, heading):
        # extract the fields from the proxy list, see find_proxies for format
        pos_field = proxy[2]
        rot_field = proxy[3]

        # set the X, Y location
        pos_field.setSFVec3f([x, y, z])

        # convert the compass heading to an angle in radians w.r.t. the reference pose and apply it to the Z rotation
        radians = (90 - heading) * (math.pi/180.0)
        rot_field.setSFRotation([0, 0, 1, radians])
        return

    #================================================================
    # Declare some MQTT callbacks.
    def on_connect(self, client, userdata, flags, rc):
        # Subscribing in on_connect() means that if we lose the connection and reconnect then subscriptions will be renewed.
        global mqtt_client
        print("%s: MQTT connected to server with with flags: %s, result code: %s" % (self.robot_name, flags, rc))
        print("%s: MQTT subscribing to %s" % (self.robot_name, self.subscription))
        self.mqtt_client.subscribe(self.subscription)

    def on_message(self, client, userdata, msg):
        # filter out our own transmissions
        if msg.topic != self.send_topic:
            # print("MQTT message received: {%s} %s" % (msg.topic, msg.payload))

            # the expected topic has the form party/name, this will extract the name
            topic_path = msg.topic.split('/')
            if len(topic_path) == 2:
                remote_name = topic_path[1]
                proxy = self.get_proxy(remote_name)
                if proxy is None:
                    printf("%s: no proxy body available for %s" % (self.robot_name, remote_name))
                else:
                    # if a proxy body is available, parse the message text
                    tokens = msg.payload.split()
                    if len(tokens) != 4:
                        print("%s received malformed message on %s: %s" % (self.robot_name, msg.topic, msg.payload))
                    else:
                        try:
                            # apply the received data to the proxy body
                            x, y, z, heading = [float(x) for x in tokens]
                            self.set_proxy_pose(proxy, x, y, z, heading)

                            # and also rebroadcast on the local radio network
                            # emitter requires a bytestring, not a Python Unicode string
                            data = remote_name.encode() + b" " + msg.payload
                            self.emitter.send(data)

                        except ValueError:
                            print("%s received malformed message on %s: %s" % (self.robot_name, msg.topic, msg.payload))

    def connect(self):
        # Connect to the MQTT network
        print("%s: initiating MQTT connection to %s:%d" % (self.robot_name, HOST, PORT))
        self.mqtt_client.connect(host=HOST, port=PORT)

    #================================================================
    def poll_receiver(self):
        """Process all available radio messages, forwarding select messages to the MQTT
           network based on the first message token."""
        while self.receiver.getQueueLength() > 0:
            packet = self.receiver.getData()
            tokens = packet.split()
            if len(tokens) < 2:
                print("%s malformed packet: %s" % (self.robot_name, packet))
            else:
                name = tokens[0].decode() # convert bytestring to Unicode
                if name == self.public_robot:
                    # retransmit the remaining message string unchanged over MQTT; this
                    # will retain some flexibility against message format changes
                    data = b" ".join(tokens[1:])
                    self.mqtt_client.publish(self.send_topic, data)

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

            # Poll the MQTT network system.
            self.mqtt_client.loop(timeout=0.0)

            # Read simulator clock time.
            t = self.getTime()

            # Test: move one proxy


        # Shut down the networking cleanly.  At this point
        # print() output will no longer show in the Webots
        # console, so this is silent.
        self.mqtt_client.disconnect()

################################################################

controller = Disembod()
controller.connect()
controller.run()
