from argparse import ArgumentParser
import time
import serial
import struct

parser = ArgumentParser()
parser.add_argument("com_port", type=str)
parser.add_argument("cmd", type=str, choices=["MVP", "MST", "GAP", "SAP"])

parser.add_argument("-v", "--value", type = int)
parser.add_argument("-t", "--type", type = str)

args = parser.parse_args()


def add_checksum(packed_data):
    byte_data = struct.unpack("8B",packed_data)
    checksum  = sum(byte_data) & 0xff
    return struct.pack( "9B", *byte_data, checksum )

axis_params = dict(
    AP_TARGET_POS = (0, "i"),
    AP_ACTUAL_POS = (1, "i"),
    AP_TARGET_VEL = (2, "i"),
    AP_ACTUAL_VEL = (3, "i"),
    AP_MAX_VEL = (4, "I"),
    AP_MAX_ACC = (5, "I"),
    AP_POS_REACHED = (8, "I"),
    AP_HOME_SW_STATE = (9, "I"),
    AP_RIGHT_SW_STATE = (10, "I"),
    AP_LEFT_SW_STATE = (11, "I"),
    AP_RIGHT_SW_DISABLE = (12, "I"),
    AP_LEFT_SW_DISABLE = (13, "I"),
    AP_SWAP_LIMITS = (14, "I"),
    AP_START_VEL = (19, "I"),
    AP_RIGHT_SW_POLAR = (24, "I"),
    AP_LEFT_SW_POLAR = (25, "I"),
    AP_MICROSTEP_RESOLUTION = (140, "I"),
    AP_ENDS_DISTANCE = (196, "I"),
    AP_REVERSE_SHAFT = (251, "I"),
)
ser = serial.Serial(args.com_port, 115200, timeout=2)

if args.cmd == "GAP":
    for name, (pid, pid_type) in axis_params.items(): 
        cmd_data = struct.pack( f">BBBB{pid_type}", 0,6,pid,0,0 )
        ser.write(add_checksum(cmd_data))

        reply = ser.read(9)
        _,_,sc,_,v,_ = struct.unpack( f'>BBBB{pid_type}B', reply )
        print( f"{name}: {v if sc == 100 else f'ERR({sc})'}" )

elif args.cmd == "MVP":
    target = args.value
    cmd_data = struct.pack( ">BBBBi", 0,4,0,0,target )
    ser.write(add_checksum(cmd_data))

    reply = ser.read(9)
    print( f"{struct.unpack( '>BBBBIB', reply )}" )

elif args.cmd == "SAP":
    name = args.type
    pid, pid_type = axis_params[name]
    cmd_data = struct.pack( f">BBBB{pid_type}", 0,5,pid,0,args.value )
    ser.write(add_checksum(cmd_data))

    reply = ser.read(9)
    _,_,sc,_,v,_ = struct.unpack( f'>BBBB{pid_type}B', reply )
    print( f"{name}: {v if sc == 100 else f'ERR({sc})'}" )

elif args.cmd == "MST":
    target = args.value
    cmd_data = struct.pack( ">BBBBi", 0,3,0,0,0 )
    ser.write(add_checksum(cmd_data))

    reply = ser.read(9)
    print( f"{struct.unpack( '>BBBBIB', reply )}" )
