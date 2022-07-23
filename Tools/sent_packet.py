from argparse import ArgumentParser
import time
import serial
import struct

parser = ArgumentParser()
parser.add_argument("com_port", type=str)
parser.add_argument("cmd", type=str, choices=["MVP", "GAP"])

parser.add_argument("-v", "--value", type = int)

args = parser.parse_args()

axis_params = dict(
    AP_TARGET_POS = 0,
    AP_ACTUAL_POS = 1,
    AP_TARGET_VEL = 2,
    AP_ACTUAL_VEL = 3,
    AP_MAX_VEL = 4,
    AP_MAX_ACC = 5,
    AP_POS_REACHED = 8,
    AP_HOME_SW_STATE = 9,
    AP_RIGHT_SW_STATE = 10,
    AP_LEFT_SW_STATE = 11,
    AP_RIGHT_SW_DISABLE = 12,
    AP_LEFT_SW_DISABLE = 13,
    AP_SWAP_LIMITS = 14,
    AP_RIGHT_SW_POLAR = 24,
    AP_LEFT_SW_POLAR = 25,
    AP_MICROSTEP_RESOLUTION = 140,
    AP_ENDS_DISTANCE = 196,
    AP_REVERSE_SHAFT = 251,
)
ser = serial.Serial(args.com_port, 115200, timeout=2)

if args.cmd == "GAP":
    for name, pid in axis_params.items(): 
        cmd_data = struct.pack( ">BBBBIB", 0,6,pid,0,0,0 )
        ser.write(cmd_data)

        reply = ser.read(9)
        print( f"{name}: {struct.unpack( '>BBBBIB', reply )}" )

elif args.cmd == "MVP":
    target = args.value
    cmd_data = struct.pack( ">BBBBiB", 0,4,0,0,target,0 )
    ser.write(cmd_data)

    reply = ser.read(9)
    print( f"{struct.unpack( '>BBBBIB', reply )}" )