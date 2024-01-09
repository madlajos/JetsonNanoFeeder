import snap7
from snap7.util import *
import struct
import array as arr

# Constants for PLC DB
db_number = 2
WriteBuffLength = 12

# Operation state of the PLC. 0-Idle, 1-Feeding
op_state_pos = 0
op_state_size = 2

# Data transmission frequency to PLC of measured BlobVolume in ms 
data_freq_pos = 2
data_freq_size = 2

# Measured BlobVolume
blob_volume_pos = 4
blob_volume_size = 4

# Byte for communication state polling
# First bit is toggled by PLC, second bit is toggled by Jetson
comm_state_pos = 8
comm_state_size = 2

# Byte for sending error codes to PLC
error_code_pos = 10
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
    try:
        data = plc.db_read(db_number, 0, 12)
        op_state, data_frequency, blob_volume, comm_state, error_code = struct.unpack(">hhIHH", data)

        return op_state, comm_state, data_frequency

    except Exception as e:
        print("Error reading from PLC:", str(e))
        return None

def getCameraParamsFromPLC(plc):
    try:
        data = plc.db_read(db_number, 0, 32)
        
        print(data)
        data2 = data[12:32]
        print(data2)
        image_width, image_height, exposure_time, gain, gamma, x_offset, y_offset, threshold_low, threshold_high, background = struct.unpack(">hhhhhhhhhh", data2)

        return image_width, image_height, exposure_time, gain, gamma, x_offset, y_offset, threshold_low, threshold_high, background

    except Exception as e:
        print("Error reading from PLC:", str(e))
        return None


def sendCommByte(plc, commByte):
    db_w_buffer = bytearray(2)
    struct.pack_into(">H", db_w_buffer, 0, commByte)
    plc.db_write(db_number, 8, db_w_buffer)
    print("CommByte Switched to: " + str(commByte)) 

def SendBlobVolume(plc, cummulative_volume):
    db_w_buffer = bytearray(4)
    struct.pack_into(">I", db_w_buffer, 0, int(cummulative_volume))
    plc.db_write(db_number, 4, db_w_buffer)
    print("BlobVolume Sent: " + str(cummulative_volume)) 

def SendPLCError(plc, error_code):
    db_w_buffer = bytearray(2)
    struct.pack_into(">H", db_w_buffer, 0, error_code)
    plc.db_write(db_number, 10, db_w_buffer)
    print("PLC Error sent: " + str(error_code))