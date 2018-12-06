# Quick sketch that sets Mosquito parameters via MSP
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
parser.add_argument('-c','--constants', type=tuple, action='store', help="Tuple containing PID constants", default=(0.06, 0.01, 0.00, 0.06, 0.01, 8.00, 0.3))

args = parser.parse_args()


def set_positioning_board(has_positioning_board, serial_com, print_data = True):
	data = serialize_SET_POSITIONING_BOARD(has_positioning_board);
	serial_com.write(data)
	if print_data:
		print("Data to send: {}".format(data))

def set_mosquito_version(mosquito_version, serial_com, print_data = True):
	data = serialize_SET_MOSQUITO_VERSION(mosquito_version);
	serial_com.write(data)
	if print_data:
		print("Data to send: {}".format(data))

def set_rate_pid(constants, serial_com, print_data = True):
	"""
	Constants should be a tuple of 6 values
	"""
	data = serialize_SET_PID_CONSTANTS(*constants)
	serial_com.write(data)
	if print_data:
		print("Data to send: {}".format(data))

def main(pos_board_param, mosquito_version_param, constants_param):
	# Serial communication object
	serial_com = serial.Serial(PORT, BAUDRATE)
	if pos_board_param is not None:
		set_positioning_board(pos_board_param, serial_com)
	if mosquito_version_param is not None:
		set_mosquito_version(mosquito_version_param, serial_com)
	set_rate_pid(constants_param, serial_com)


if __name__ == '__main__':
	main(args.position, args.mosquito, args.constants)
