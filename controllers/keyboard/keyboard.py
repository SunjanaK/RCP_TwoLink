# keyboard.py

# Sample Webots controller file for running the piano keyboard robot.

# No copyright, 2020, Garth Zeglin.  This file is
# explicitly placed in the public domain.

print("loading keyboard.py...")

# Import the Webots simulator API.
from controller import Robot

# Import standard Python libraries.

# Try setting up the MIDI interface.
# See https://spotlightkid.github.io/python-rtmidi/
try:
    import rtmidi
    _midi_available = True
    
except ModuleNotFoundError:
    print("rtmidi module not found, direct MIDI output not available.")
    _midi_available = False
    
# this should be customized for your particular MIDI synth
preferred_MIDI_device = 'IAC'

# define a C major scale for the keyboard
note_mapping = [60, 62, 64, 65, 67, 69, 71, 72]

# Define the time step in milliseconds between controller updates.
EVENT_LOOP_DT = 200

################################################################
class Keyboard(Robot):
    def __init__(self):

        super(Keyboard, self).__init__()
        self.robot_name = self.getName()
        print("%s: controller connected." % (self.robot_name))

        if _midi_available:
            self.midiout = rtmidi.MidiOut()
            available_ports = self.midiout.get_ports()
            print("%s: available MIDI ports: %s" % (self.robot_name, available_ports))
            for number, name in enumerate(available_ports):
                if preferred_MIDI_device in name:
                    self.midiout.open_port(number)
                    print("%s: opened MIDI output on  %s" % (self.robot_name, name))
                    break

        # This needs to be customized to match the model:
        self.num_keys = 8
                
        # Fetch handles for the key sensors.
        self.key_sensors = [self.getDevice('key' + str(idx+1)) for idx in range(self.num_keys)]

        # Specify the sampling rate for the joint sensors.
        for key in self.key_sensors:
            key.enable(EVENT_LOOP_DT)

        # State array for detecting changes.
        self.key_previous = [False for key in self.key_sensors]
        return

    #================================================================
    def run(self):
        # Run loop to execute a periodic script until the simulation quits.
        # If the controller returns -1, the simulator is quitting.
        while self.step(EVENT_LOOP_DT) != -1:
            # Read simulator clock time.
            # self.sim_time = self.getTime()

            # Read keyboard values.
            positions = [key.getValue() for key in self.key_sensors]

            # Quantize the angle
            pressed = [value > 0.05 for value in positions]

            # Check for changes.
            changes = [press ^ previous for press, previous in zip(pressed, self.key_previous)]
            self.key_previous = pressed
            
            if any(changes):
                print("%s: keyboard state: %s" % (self.robot_name, pressed))

            if _midi_available and self.midiout.is_port_open():
                for num, changed in enumerate(changes):
                    if changed:
                        if pressed[num]:
                            # generate a 'NoteOn' on channel 1
                            msg = [0x90, note_mapping[num], 112]
                        else:
                            # generate a 'NoteOff' on channel 1
                            msg = [0x80, note_mapping[num], 0]
                        self.midiout.send_message(msg)
                
################################################################
# Start the script.
robot = Keyboard()
robot.run()
