import snap7
from snap7.util import *
import struct

ReadBlock = 23
WriteBlock = 30
ReadBuffLength = 16
WriteBuffLength = 21

# Constants for ReadFeeder
RPMPos = 0
RPMSize = 4

FeedRatePos = 4
FeedRateSize = 4

MassPos = 8
MassSize = 4

VersionPos = 12
VersionSize = 4

# Constants for WriteFeeder
RPMPos = 0
FeedRatePos = 4
RPM1Pos = 8
RPM2Pos = 12
OpModePos = 16
StartRefillPos = 18
StartPos = 19
StopPos = 20

def PollPLC(plc):
    feeder_state = 1
    commstate = 1
    plc_data_frequency = 500


    return feeder_state, commstate, plc_data_frequency

def SendBlobVolume(plc, cummulative_volume):
    print("")

    


""" def GetFeederVersion(plc):
    #plc = snap7.client.Client()
    #plc.connect(plc_ip, 0, 1)
    
    data = plc.db_read(ReadBlock, VersionPos, VersionSize)
    version_int = struct.unpack(">I", data)[0]
    plc_version = struct.unpack("f", struct.pack("I", version_int))[0]
    #plc.disconnect()

    return plc_version


def GetFeederMass(plc):
    #plc = snap7.client.Client()
    #plc.connect(plc_ip, 0, 1)
    
    data = plc.db_read(ReadBlock, MassPos, MassSize)
    mass_int = struct.unpack(">I", data)[0]
    plc_mass = struct.unpack("f", struct.pack("I", mass_int))[0]
    #plc.disconnect()
    #plc_mass = 0
    return plc_mass

# TODO: Needs Testing
def start_feeder(plc):
    db_w_buffer = bytearray(21)
    snap7.util.set_bool(db_w_buffer, self.StartPos, True)

    plc.client.db_write(WriteBlock, 0, db_w_buffer)

# TODO: Needs Testing
def stop_feeder(plc):
    db_w_buffer = bytearray(21)
    snap7.util.set_bool(db_w_buffer, plc.StartPos, False)

    plc.client.db_write(self.WriteBlock, 0, db_w_buffer)

# TODO: Needs Testing
def SendFeederSpeed(plc, feeder_speed):
    db_w_buffer = bytearray(21)
    if feeder_speed is not None:
        # Set the real value in the buffer
        struct.pack_into(">f", db_w_buffer, RPMPos, feeder_speed)

         # Write the data to the PLC
        plc.db_write(WriteBlock, 0, db_w_buffer) """