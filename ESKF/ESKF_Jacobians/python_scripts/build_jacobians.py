# Script to build Jacobians for BonaDrone's ESKF implementation 
#
# Author: Juan Gallostra jgallostra<at>bonadrone.com
# Date: 01-09-2018
#
# Steps:
#  1. Run the Matlab script that builds the nominal and error state jacobians (Jacobians.m)
#  2. Copy the desired Jacobian in a plain text file (.txt)
#  3. Update the Matlab <-> Hackflight relations in correspondences.py
#  4. Run this script with the previous text file as input 
#

import substitutions as s
import argparse

# Command line arguments
parser = argparse.ArgumentParser()

parser.add_argument('-i','--input', type=str, action='store', help="input text file")
parser.add_argument('-o','--output', type=str, action='store', help="output text file")
parser.add_argument('-n','--name', type=str, action='store', help="jacobian name")

args = parser.parse_args()

def main(input_file, output_file, jacobian_name = "Fx"):
	content = None
	rows = 0
	cols = 0
	# read data as computed with Matlab
	with open(input_file) as f:
		content = [x.strip() for x in f.readlines()]
	rows = len(content)
	content = [i.split(",") for i in content]
	cols = len(content[0])
	# change powers and var names
	jacobian = []
	for line in content:
		curr_row = []
		for element in line:
			if element[0] == "[":
				element = element[1:]
			if element[-1] == "]":
				element = element[:-1]
			replaced_powers = s.replace_powers(element)
			mod_element = s.change_var_names(replaced_powers)
			curr_row += [mod_element]
		jacobian += [curr_row]
	# build jacobian in Hackflight format
	for i in range(rows):
		for j in range(cols):
			jacobian[i][j] = jacobian_name+"["+str(i*cols+j)+"] = "+jacobian[i][j]+";\n"
	# Write to a txt file:
	with open(output_file, 'w') as file:
		for j in range(cols):
			file.write("// "+str(j+1)+" column\n")
			for i in range(rows):
				file.write(jacobian[i][j])

if __name__ == '__main__':
	main(args.input, args.output, args.name)
