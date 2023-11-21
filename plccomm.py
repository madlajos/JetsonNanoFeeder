import snap7
from snap7.util import *
import struct

# Constants for PLC DB
db_number = 2
WriteBuffLength = 21

# Operation state of the PLC. 0-Idle, 1-Feeding
op_state_pos = 0
op_state_size = 4

# Data transmission frequency to PLC of measured BlobVolume in ms 
data_freq_pos = 4
data_freq_size = 4

# Measured BlobVolume
blob_volume_pos = 8
blob_volume_size = 4

# Byte for communication state polling
# First bit is toggled by PLC, second bit is toggled by Jetson
comm_state_pos = 12
comm_state_size = 2

# Byte for sending error codes to PLC
error_code_pos = 14
error_code_size = 2

def connectToPLC(plc_ip):
    plc = snap7.client.Client()
 
    try:
        plc.connect(plc_ip, 0, 1)
        connected_to_plc = True
    except Exception as e:
        print("Failed to connect to PLC:", str(e))
        plc = None
    
    return plc, connected_to_plc 

def PollPLC(plc):
    data = plc.db_read(db_number, op_state_pos, op_state_size)
    op_state = struct.unpack(">I", data)[0]
    
    data = plc.db_read(db_number, data_freq_pos, data_freq_size)
    data_frequency = struct.unpack(">I", data)[0]
    
    data = plc.db_read(db_number, comm_state_pos, comm_state_size)
    commstate = struct.unpack("B", data)[0]

    return op_state, commstate, data_frequency

def sendCommByte(plc, commByte):
    db_w_buffer = bytearray(WriteBuffLength)
    struct.pack_into("B", db_w_buffer, comm_state_pos, commByte)
    plc.db_write(db_number, 0, db_w_buffer)

def SendBlobVolume(plc, cummulative_volume):
    db_w_buffer = bytearray(WriteBuffLength)
    struct.pack_into(">f", db_w_buffer, blob_volume_pos, cummulative_volume)
    plc.db_write(db_number, 0, db_w_buffer)

def SendPLCError(plc, error_code):
    db_w_buffer = bytearray(WriteBuffLength)
    struct.pack_into("B", db_w_buffer, error_code_pos, error_code_size)
    plc.db_write(db_number, 0, db_w_buffer)