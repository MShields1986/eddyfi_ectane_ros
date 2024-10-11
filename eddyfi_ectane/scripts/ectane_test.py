from enum import Enum
from dataclasses import dataclass
import socket
import time

# In line with Euan Foster's additions to InstDefsEctIris.h
# https://github.com/UoS-EEE-Automation/EctaneLabviewIntegration/blob/3bd3d2f49d6a7a6e077b98b8bf7a1289c4204995/C%20Programs/InstDefsEctIris.h#L1443

class State(Enum):
    DO_NOTHING = 0
    SOCKET_INITIALISE = 1
    CONNECT_TO_DEVICE = 2
    SET_UP_DEVICE = 3
    BALANCE_DEVICE = 4
    ACQUIRE_DATA = 5
    STOP_DATA = 6
    DISCONNECT_DEVICE = 7

@dataclass
class CommandPacket:
    state: int
    ip_address: str
    configuration: int
    acquisition_rate: int
    gain: int
    frequency_1: int
    voltage_1: float
    frequency_2: int
    voltage_2: float
    frequency_3: int
    voltage_3: float
    frequency_4: int
    voltage_4: float
    frequency_5: int
    voltage_5: float
    frequency_6: int
    voltage_6: float
    frequency_7: int
    voltage_7: float
    frequency_8: int
    voltage_8: float

# In line with Euan Foster's Configurations.h
# https://github.com/UoS-EEE-Automation/EctaneLabviewIntegration/blob/3bd3d2f49d6a7a6e077b98b8bf7a1289c4204995/C%20Programs/Configurations.h
class Configurations(Enum):
    EXAMPLE = 0
    HF_PROBE_AXIAL_1_CHANNEL = 1
    HF_PROBE_TRANSVERSAL_1_CHANNEL = 2
    HF_PROBE_AXIAL_AND_TRANSVERSAL_1_CHANNEL = 3
    HF_PROBE_AXIAL_AND_TRANSVERSAL_4_CHANNELS = 4
    LF_PROBE_AXIAL_1_CHANNEL = 5
    LF_PROBE_TRANSVERSAL_1_CHANNEL = 6
    LF_PROBE_AXIAL_AND_TRANSVERSAL_1_CHANNEL = 7
    LF_PROBE_AXIAL_AND_TRANSVERSAL_4_CHANNELS = 8


class EctaneHandler:
    def __init__(
        self, 
        ectane_ip: str,
        configuration: Configurations, 
        acquisition_rate: int, 
        gain: int, 
        frequency: int, 
        voltage: float
        ):

        print("Initialising EctaneHandler...")

        self.state = State.DO_NOTHING
        self.ectane_ip = ectane_ip
        self.configuration = configuration
        self.acquisition_rate = acquisition_rate
        self.gain = gain
        self.frequency = frequency
        self.voltage = voltage

        self.update_command_packet()

        print("Starting the TCP servers...")
        self.data_receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.data_receiver_socket.bind(('127.0.0.1', 24011))

        self.command_sender_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.command_sender_socket.bind(('127.0.0.1', 24010))

        print("Please start the C SDK now")

        self.data_receiver_socket.listen(10)
        self.command_sender_socket.listen(10)

        self.data_receiver, self.data_receiver_address = self.data_receiver_socket.accept()
        self.command_sender, self.command_sender_address = self.command_sender_socket.accept()

        print("Connected to C SDK")


    def __del__(self):
        print("Cleaning up EctaneHandler...")

        self.data_receiver_socket.close()
        self.command_sender_socket.close()

        print("Exiting EctaneHandler")


    def read_data(self):
        packet_size = int((
                32                          # Timeslots per observation
                * 8                         # Channels per timeslot
                * 5                         # Fields per channel (5 Freqs + 1 Voltage)
                #+ 3                        # Encoders    --- NOT FORWARDED FROM SDK ATM
                #+ 32 / 4                   # Saturations --- NOT FORWARDED FROM SDK ATM
                #+ 1                        # Metadata    --- NOT FORWARDED FROM SDK ATM
            )
            * 32                        # Bits per field
            / 8                         # Bits per byte
            )
        
        self.ec_data = self.data_receiver.recv(packet_size)


    def update_command_packet(self):
        self.command_packet = CommandPacket(
            state = self.state,
            ip_address = self.ectane_ip,
            configuration = self.configuration,
            acquisition_rate = self.acquisition_rate,
            gain = self.gain,
            frequency_1   = self.frequency,
            voltage_1 = self.voltage,
            frequency_2 = self.frequency,
            voltage_2 = self.voltage,
            frequency_3 = self.frequency,
            voltage_3 = self.voltage,
            frequency_4 = self.frequency,
            voltage_4 = self.voltage,
            frequency_5 = self.frequency,
            voltage_5 = self.voltage,
            frequency_6 = self.frequency,
            voltage_6 = self.voltage,
            frequency_7 = self.frequency,
            voltage_7 = self.voltage,
            frequency_8 = self.frequency,
            voltage_8 = self.voltage
        )

        self.update_command_string()


    def update_command_string(self):
        self.command_string = f"{self.command_packet.state.value},{self.command_packet.ip_address},{self.command_packet.configuration.value},{self.command_packet.acquisition_rate},{self.command_packet.gain},{self.command_packet.frequency_1},{self.command_packet.voltage_1},{self.command_packet.frequency_2},{self.command_packet.voltage_2},{self.command_packet.frequency_3},{self.command_packet.voltage_3},{self.command_packet.frequency_4},{self.command_packet.voltage_4},{self.command_packet.frequency_5},{self.command_packet.voltage_5},{self.command_packet.frequency_6},{self.command_packet.voltage_6},{self.command_packet.frequency_7},{self.command_packet.voltage_7},{self.command_packet.frequency_8},{self.command_packet.voltage_8}\0"


    def change_state(self, state: State):
        self.state = state
        self.update_command_packet()
        print(f"Changing state to {state.name}")
        print(f"Sending command: {self.command_string}")
        self.command_sender.send(self.command_string.encode())
        time.sleep(2)


    def start_data_acquisition(self):
        self.change_state(State.ACQUIRE_DATA)

        buffer = ''
        while True:
            self.read_data()
            if self.ec_data:
                buffer = self.ec_data
                print(buffer)
                #time.sleep(1 / self.acquisition_rate)
            else:
                break


if __name__ == '__main__':
    # Initialise handler
    handler = EctaneHandler(
        ectane_ip = "192.168.2.35",
        configuration = Configurations.LF_PROBE_AXIAL_AND_TRANSVERSAL_1_CHANNEL,
        acquisition_rate = 100,
        gain = 23,
        frequency = 5000,
        voltage = 7
        )

    # Initialise C SDK
    print("Setting up the Ectane via the C SDK...")
    handler.change_state(State.DO_NOTHING)
    handler.change_state(State.CONNECT_TO_DEVICE)
    handler.change_state(State.SET_UP_DEVICE)
    handler.change_state(State.DO_NOTHING)

    # Balance
    print("Balancing device...")
    handler.change_state(State.BALANCE_DEVICE)
    time.sleep(5)

    # GO!
    print("Starting data acquisition...")
    handler.start_data_acquisition()

    # Stop
    #handler.change_state(State.STOP_DATA)

    # Disconnect
    #handler.change_state(State.DISCONNECT_DEVICE)

    # Clean up
    #delete(handler)
