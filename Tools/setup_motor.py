import os 
import subprocess
import sys

script_path = os.path.dirname(os.path.realpath(__file__))
if len(sys.argv) > 1:
    com_port = sys.argv[1]
else:
    com_port = "COM3"

subprocess.call( [ 
    "python",
    os.path.join( script_path, "send_packet.py" ),
    com_port,
    "SAP",
    "-t",
    "AP_MAX_VEL",
    "-v",
    "5000"
 ] )

subprocess.call( [ 
    "python",
    os.path.join( script_path, "send_packet.py" ),
    com_port,
    "SAP",
    "-t",
    "AP_MAX_ACC",
    "-v",
    "1000"
 ] )

subprocess.call( [ 
    "python",
    os.path.join( script_path, "send_packet.py" ),
    com_port,
    "SAP",
    "-t",
    "AP_START_VEL",
    "-v",
    "500"
 ] )

subprocess.call( [ 
    "python",
    os.path.join( script_path, "send_packet.py" ),
    com_port,
    "SAP",
    "-t",
    "AP_MICROSTEP_RESOLUTION",
    "-v",
    "4"
 ] )
