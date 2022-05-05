'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 4 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_4.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

from tarfile import TarFile
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
from pyzbar.pyzbar import decode

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()

try:
	import task_1b

except ImportError:
	print('\n[ERROR] task_1b.py file is not present in the current directory.')
	print('Your current directory is: ', os.getcwd())
	print('Make sure task_1b.py is present in this current directory.\n')
	sys.exit()

try:
	import task_2a

except ImportError:
	print('\n[ERROR] task_1b.py file is not present in the current directory.')
	print('Your current directory is: ', os.getcwd())
	print('Make sure task_1b.py is present in this current directory.\n')
	sys.exit()

try:
	import task_3

except ImportError:
	print('\n[ERROR] task_1b.py file is not present in the current directory.')
	print('Your current directory is: ', os.getcwd())
	print('Make sure task_1b.py is present in this current directory.\n')
	sys.exit()


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################


##############################################################

def init_remote_api_server():
    client_id =-1    
    sim.simxFinish(-1)
    client_id = sim.simxStart('127.0.0.1',19997,True,True,5000,5)
    return client_id


def start_simulation(client_id):
	return_code = -2
	if client_id != -1 :
		return_code = sim.simxStartSimulation(client_id,sim.simx_opmode_blocking)

	return return_code

def stop_simulation(client_id):
	return_code = -2
	return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
	return return_code


def exit_remote_api_server(client_id):
	sim.simxFinish(client_id)

def pluck(client_id,berry_positions_dictionary,vision_sensor_handle):
	return_code, box = sim.simxGetObjectHandle(client_id, 'container_box_1', sim.simx_opmode_blocking)
	return_code, pos = sim.simxGetObjectPosition(client_id, box,vision_sensor_handle, sim.simx_opmode_blocking)
	B = berry_positions_dictionary
	return_code, target = sim.simxGetObjectHandle(client_id, 'robotic_arm_target#0', sim.simx_opmode_blocking)
	for x in B:
		return_code,int1,float1,string1,buffer1=sim.simxCallScriptFunction(client_id, 'gripper', sim.sim_scripttype_childscript, 'open_close', [], [],'open',bytearray(),sim.simx_opmode_blocking)
		time.sleep(0.5)
		return_code = sim.simxSetObjectPosition(client_id,target, vision_sensor_handle,B[x][0],sim.simx_opmode_blocking)
		time.sleep(0.5)
		return_code,int2,float2,string2,buffer2=sim.simxCallScriptFunction(client_id, 'gripper', sim.sim_scripttype_childscript, 'open_close', [], [],'close',bytearray(), sim.simx_opmode_blocking)
		time.sleep(0.5)
		return_code = sim.simxSetObjectPosition(client_id,target, vision_sensor_handle,pos,sim.simx_opmode_blocking)
		time.sleep(0.5)





def berries_detection(client_id):
	while True:
		return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
		vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
		vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
		transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
		transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
		if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):
			berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
			print("Berries Dictionary = ", berries_dictionary)
			berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
			pluck(client_id,berry_positions_dictionary,vision_sensor_handle)
			print("Berry Positions Dictionary = ",berry_positions_dictionary)
		if(len(berry_positions_dictionary)>0):
			break

def task_4_primary(client_id):
	target_points=[(2,2),(4,3),(4,4)]
	task_3.task_3_primary(client_id, target_points)
	berries_detection(client_id)
	# return_code, rel_dum = sim.simxGetObjectHandle(client_id, 'Dummy2', sim.simx_opmode_blocking)
	# return_code, dummy = sim.simxGetObjectHandle(client_id, 'Dummy1', sim.simx_opmode_blocking)
	# # return_code, lemon1 = sim.simxGetObjectHandle(client_id, 'lemon_1', sim.simx_opmode_blocking)
	# # return_code, lemon3 = sim.simxGetObjectHandle(client_id, 'lemon_3', sim.simx_opmode_blocking)
	# # return_code, lemon4 = sim.simxGetObjectHandle(client_id, 'lemon_4', sim.simx_opmode_blocking)
	# return_code, target = sim.simxGetObjectHandle(client_id, 'robotic_arm_target#0', sim.simx_opmode_blocking)
	
	# # return_code, posi1 = sim.simxGetObjectPosition(client_id, lemon1,rel_dum, sim.simx_opmode_blocking)
	# # return_code, posi3 = sim.simxGetObjectPosition(client_id, lemon3,rel_dum, sim.simx_opmode_blocking)
	# # return_code, posi4 = sim.simxGetObjectPosition(client_id, lemon4,rel_dum, sim.simx_opmode_blocking)
	# print(pos)
	# return_code = sim.simxSetObjectPosition(client_id,target, rel_dum,pos,sim.simx_opmode_blocking)
	# time.sleep(3)
	"""
	Purpose:
	---
	This is the only function that is called from the main function. Make sure to fill it
	properly, such that the bot traverses to the vertical rack, detects, plucks & deposits a berry of each color.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	
	Returns:
	---
	
	Example call:
	---
	task_4_primary(client_id)
	
	"""


if __name__ == "__main__":


	##################################################
	## You are NOT allowed to make any changes in the code below ##

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

			# Starting the Simulation
			try:
				return_code = start_simulation(client_id)

				if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
					print('\nSimulation started correctly in CoppeliaSim.')

				else:
					print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
					print('start_simulation function is not configured correctly, check the code!')
					print()
					sys.exit()

			except Exception:
				print('\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
				print('Stop the CoppeliaSim simulation manually.\n')
				traceback.print_exc(file=sys.stdout)
				print()
				sys.exit()

		else:
			print('\n[ERROR] Failed connecting to Remote API server!')
			print('[WARNING] Make sure the CoppeliaSim software is running and')
			print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
			print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
			print()
			sys.exit()

	except Exception:
		print('\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()

	try:

		task_4_primary(client_id)
		time.sleep(1)        

		try:
			return_code = stop_simulation(client_id)                            
			if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
				print('\nSimulation stopped correctly.')

				# Stop the Remote API connection with CoppeliaSim server
				try:
					exit_remote_api_server(client_id)
					if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
						print('\nDisconnected successfully from Remote API Server in CoppeliaSim!')

					else:
						print('\n[ERROR] Failed disconnecting from Remote API server!')
						print('[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

				except Exception:
					print('\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
					print('Stop the CoppeliaSim simulation manually.\n')
					traceback.print_exc(file=sys.stdout)
					print()
					sys.exit()
									  
			else:
				print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
				print('[ERROR] stop_simulation function is not configured correctly, check the code!')
				print('Stop the CoppeliaSim simulation manually.')
		  
			print()
			sys.exit()

		except Exception:
			print('\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

	except Exception:
		print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()