import snap7
from snap7.util import *
import struct

ReadBlock = 23
WriteBlock = 30
ReadBuffLength = 16
WriteBuffLength = 21

plc_ip = "192.168.0.30"

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

def GetFeederVersion(plc):
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

