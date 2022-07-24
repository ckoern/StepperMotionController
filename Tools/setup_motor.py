import os 
import subprocess

script_path = os.path.dirname(os.path.realpath(__file__))
com_port = "COM3"
subprocess.call( [ 
    "python",
    os.path.join( script_path, "send_packet.py" ),
    com_port,
    "SAP",
    "-t",
    "AP_MAX_VEL",
    "-v",
    "500"
 ] )

subprocess.call( [ 
    "python",
    os.path.join( script_path, "send_packet.py" ),
    com_port,
    "SAP",
    "-t",
    "AP_MAX_ACC",
    "-v",
    "100"
 ] )

subprocess.call( [ 
    "python",
    os.path.join( script_path, "send_packet.py" ),
    com_port,
    "SAP",
    "-t",
    "AP_START_VEL",
    "-v",
    "100"
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
