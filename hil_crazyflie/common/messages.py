import struct

# Motor commands: 4 floats
CMD_FMT = "ffff"
STATE_FMT = "ffffff"  # x, y, z, roll, pitch, yaw

def pack_cmd(cmd):
    return struct.pack(CMD_FMT, *cmd)

def unpack_cmd(data):
    return struct.unpack(CMD_FMT, data)

def pack_state(state):
    return struct.pack(STATE_FMT, *state)

def unpack_state(data):
    return struct.unpack(STATE_FMT, data)