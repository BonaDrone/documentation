# Quick sketch that sets Mosquito parameters via MSP
# Requires a system wide or local installation of msppg.py.
#
# See: https://github.com/BonaDrone/Hackflight/tree/master/extras/parser
#
# Author: Juan Gallostra
# Date: 05-12-2018

import serial
import argparse
from msppg import serialize_SET_PID_CONSTANTS, serialize_SET_POSITIONING_BOARD, serialize_SET_MOSQUITO_VERSION

PORT        = '/dev/ttyACM0'
BAUDRATE    = 9600


# Command line arguments
parser = argparse.ArgumentParser(description='Mosquito parameter set via MSP')

parser.add_argument('-m','--mosquito', type=int, action='store', help="Mosquito version (1->90, 0->150)")
parser.add_argument('-p','--position', type=int, action='store', help="Positioning board present (1/0)")
#parser.add_argument('-c','--constants', type=tuple, action='store', help="Tuple containing PID constants")

args = parser.parse_args()


def set_positioning_board(has_positioning_board, serial_com, print_data = True):
	"""
	Set if the Mosquito has the positoning board (1 if True and 0 if False)
	and send it to the Mosquito via an MSP message
	"""
	data = serialize_SET_POSITIONING_BOARD(has_positioning_board)
	serial_com.write(data)
	if print_data:
		print("Data to send: {}".format(data))

def set_mosquito_version(mosquito_version, serial_com, print_data = True):
	"""
	Set the version of the Mosquito (90 if mosquito_version is 1 and 150 if
	mosquito version is 0) and send it to the Mosquito via an MSP message
	"""
	data = serialize_SET_MOSQUITO_VERSION(mosquito_version)
	serial_com.write(data)
	if print_data:
		print("Data to send: {}".format(data))

def set_rate_pid(m90, serial_com, print_data = True):
	"""
	Set the Mosquito's PID constants via an MSP message.
	Constants should be a tuple of 7 values. In order:
	- gyroRollPitchP (Rate PID)
	- gyroRollPitchI (Rate PID)
	- gyroRollPitchD (Rate PID)
	- gyroYawP       (Rate PID)
	- gyroYawI       (Rate PID)
	- demandsToRate  (Rate PID)
	- levelP         (Level PID)
	- altHoldP       (AltHold PID)
	- altHoldVelP    (AltHold PID)
	- altHoldVelI    (AltHold PID)
	- altHoldVelD    (AltHold PID)
	- minAltitude    (AltHold PID)
	"""

	constants = None

	if m90:
		RATE_ROLL_P = 0.05
		RATE_ROLL_I = 0.40
		# RATE_ROLL_I = 0.10
		RATE_ROLL_D = 0.0001

		RATE_PITCH_P = 0.05
		RATE_PITCH_I = 0.55
		# RATE_PITCH_I = 0.15
		RATE_PITCH_D = 0.0001

		RATE_YAW_P = 0.05
		RATE_YAW_I = 0.40

		RATE_D2R = 6.00

		LEVEL_P = 3.0

		ALTH_P = 0.85
		ALTH_V_P = 0.85
		ALTH_V_I = 0.2
		ALTH_V_D = 0.085
		ALTH_MIN_A = 0.0

		POSH_V_P = 0.25
		POSH_V_I = 0.15
		POSH_V_D = 0.05
		PARAM_9 = 4.0

		constants = (RATE_ROLL_P, RATE_ROLL_I, RATE_ROLL_D, RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_YAW_P, RATE_YAW_I, RATE_D2R, LEVEL_P, ALTH_P, ALTH_V_P, ALTH_V_I, ALTH_V_D, ALTH_MIN_A, POSH_V_P, POSH_V_I, POSH_V_D, PARAM_9)


	else:
		RATE_ROLL_P = 0.03
		RATE_ROLL_I = 0.02
		RATE_ROLL_D = 0.0002

		RATE_PITCH_P = 0.03
		RATE_PITCH_I = 0.02
		RATE_PITCH_D = 0.0002

		RATE_YAW_P = 0.04
		RATE_YAW_I = 0.09

		RATE_D2R = 5.00

		LEVEL_P = 0.70
		# LEVEL_P = 1.50

		ALTH_P = 0.41
		ALTH_V_P = 0.19
		ALTH_V_I = 0.04
		ALTH_V_D = 0.008
		ALTH_MIN_A = 0.00

		POSH_V_P = 0.1
		POSH_V_I = 0.1
		POSH_V_D = 0.00
		PARAM_9 = 4.0

		constants = (RATE_ROLL_P, RATE_ROLL_I, RATE_ROLL_D, RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_YAW_P, RATE_YAW_I, RATE_D2R, LEVEL_P, ALTH_P, ALTH_V_P, ALTH_V_I, ALTH_V_D, ALTH_MIN_A, POSH_V_P, POSH_V_I, POSH_V_D, PARAM_9)

	data = serialize_SET_PID_CONSTANTS(*constants)
	serial_com.write(data)
	if print_data:
		print("Data to send: {}".format(data))

	return constants

def main(pos_board_param, mosquito_version_param):
	"""
	Connect with the Mosquito via Serial and send the parameters passed
	as command line arguments
	"""
	# Serial communication object
	m_version = ["150", "90"]
	p_board = ["NO", "YES"] 
	serial_com = serial.Serial(PORT, BAUDRATE)
	# process arguments and send appropriate messages
	if pos_board_param is not None:
		set_positioning_board(pos_board_param, serial_com)
	if mosquito_version_param is not None:
		set_mosquito_version(mosquito_version_param, serial_com)
	constants_param = set_rate_pid(mosquito_version_param, serial_com)
	# print configuration summary
	constants_param = [str(i) for i in constants_param]
	print("\n")
	print("***** Configuration summary *****")
	print("- Version: " + m_version[mosquito_version_param])
	print("- Positioning board: " + p_board[pos_board_param])
	print("- PID constants:")
	print("	* Rate:")
	print("		* Roll: Kp: " + constants_param[0] + ", Ki: " + constants_param[1] + ", Kd: " + constants_param[2])
	print("		* Pitch: Kp: " + constants_param[3] + ", Ki: " + constants_param[4] + ", Kd: " + constants_param[5])
	print("		* Yaw: Kp: " + constants_param[6] + ", Ki: " + constants_param[7] + ", Kd: 0.00")
	print("		* demands to rate: " + constants_param[8])
	print("	* Level: ")
	print("		* Roll: Kp: " + constants_param[9])
	print("		* Pitch: Kp: " + constants_param[9])
	print("	* Altitude Hold: ")
	print("		* Minimum altitude: " + constants_param[14])
	print("		* Position: Kp: " + constants_param[10])
	print("		* Velocity: Kp: " + constants_param[11] + ", Ki: " + constants_param[12] + ", Kd: " + constants_param[13])
	print("	* Position Hold: ")
	print("		* Position: Kp: " + constants_param[18] + " (Not used yet)")
	print("		* Velocity: Kp: " + constants_param[15] + ", Ki: " + constants_param[16] + ", Kd: " + constants_param[17])

if __name__ == '__main__':
	main(args.position, args.mosquito)
