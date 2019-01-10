# Matlab <-> Hackflight ESKF relations 
#
# Author: Juan Gallostra jgallostra<at>bonadrone.com
# Date: 01-09-2018

# Matlab to Hackflight substitutions
JACOBIAN = "Fx"

SUBS = {
	"dt" : "dt",
	"qw" : "x[6]", # x[0-5] are positions and velocities
	"qx" : "x[7]",
	"qy" : "x[8]",
	"qz" : "x[9]",
	"wsx" : "_rates[0]",
	"wsy" : "_rates[1]",
	"wsz" : "_rates[2]",
	"asx" : "_accels[0]", # to be defined
	"asy" : "_accels[1]",
	"asz" : "_accels[2]",
	"wbx" : "x[13]",
	"wby" : "x[14]",
	"wbz" : "x[15]",
	"abx" : "x[10]",
	"aby" : "x[11]",
	"abz" : "x[12]",
	"g": "",
}


# required constants
NORMAL_CHARS = [" ","+","-","/", "*", "(", ")"]
NUM_CHARS = [str(i) for i in range(10)]
POWER = "^"

# Helper functions for specific Matlab to Hackflight replacements
def replace_powers(element):
	"""
	Takes an individual element from the jacobian
	and replaces all the powers in the form a^n by a*a*...*a
	"""
	mod_element = ""
	power_start_idx = 0
	power_end_idx = 0
	possible_power = False
	found_power = False
	power_str = ""

	for idx, letter in enumerate(element):
		# get literal
		if letter not in NORMAL_CHARS and letter not in NUM_CHARS and letter != POWER and not found_power and not possible_power:
			possible_power = True
			power_start_idx = idx

		if not possible_power:
			mod_element += letter

		if possible_power and not found_power and (letter in NORMAL_CHARS or idx==len(element)-1):
			possible_power = False
			mod_element += element[power_start_idx:idx]+letter

		if letter == POWER:
			found_power = True
			power_end_idx = idx

		# get power and replace
		if found_power and letter in NUM_CHARS:
			power_str += letter
		if found_power and ((letter not in NUM_CHARS and letter != POWER) or (idx == len(element)-1)):
			# replace
			power = int(power_str)
			replaced = "*".join([element[power_start_idx:power_end_idx] for i in range(power)])
			if idx != len(element)-1 or letter not in NUM_CHARS:
				replaced += letter
			mod_element += replaced
			possible_power = False
			found_power = False
			power_str = ""

	return mod_element

def change_var_names(element):
	"""
	Takes a Jacobian element and replaces Matlab variable
	names by its Hackflight counterparts.

	This method should be called after replace_powers to avoid conflicts
	"""

	mod_element = ""
	var_start_idx = 0
	var_end_idx = 0

	found_var = False
	possible_var = False

	for idx, letter in enumerate(element):
		# get literal
		if letter not in NORMAL_CHARS+NUM_CHARS and not possible_var:
			# hit literal
			possible_var = True
			var_start_idx = idx

		if not possible_var:
			mod_element += letter

		# end of literal
		if possible_var and (letter in NUM_CHARS+NORMAL_CHARS or idx == len(element)-1):
			var_end_idx = idx
			found_var = True

		if found_var:
			if idx != len(element)-1:
				mod_element += SUBS[element[var_start_idx:var_end_idx]]+letter
			else:
				if letter not in NORMAL_CHARS+NUM_CHARS:
					mod_element += SUBS[element[var_start_idx:var_end_idx]+letter]
				else:
					mod_element += SUBS[element[var_start_idx:var_end_idx]]+letter
			possible_var = False
			found_var = False

	return mod_element
