'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 3 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ 2898 ]
# Author List:		[ Rushikesh rajesh patil, Anamika Kumari, Aritra Mitra, harsh raj ]
# Filename:			task_3.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
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


def get_vision_sensor_image(client_id):
    vision_sensor_image = []
    image_resolution = []
    return_code = 0
    return_code, vision_sensor = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, vision_sensor,0, sim.simx_opmode_blocking)
    return vision_sensor_image, image_resolution, return_code


def transform_vision_sensor_image(vision_sensor_image, image_resolution):
    transformed_image = None
    transformed_image = np.array(vision_sensor_image, dtype = np.uint8)
    transformed_image.resize(image_resolution[0], image_resolution[1], 3)
    transformed_image = cv2.flip(transformed_image,0)
    transformed_image =  cv2.cvtColor(transformed_image, cv2.COLOR_BGR2RGB)
    return transformed_image


def stop_simulation(client_id):
	return_code = -2
	return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
	return return_code


def exit_remote_api_server(client_id):
	sim.simxFinish(client_id)	


def detect_qr_codes(transformed_image):
    qr_codes = []

    detected = decode(transformed_image)
    for qr in detected:
        points = (qr.data).decode()
        qr_codes.append(points)
    return qr_codes


def set_bot_movement(client_id,wheel_joints,forw_back_vel,left_right_vel,rot_vel):
    sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],-forw_back_vel-left_right_vel-rot_vel,sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],-forw_back_vel+left_right_vel-rot_vel,sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],-forw_back_vel-left_right_vel+rot_vel,sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],-forw_back_vel+left_right_vel+rot_vel,sim.simx_opmode_blocking)


def init_setup(client_id):
	wheel_joints=[-1,-1,-1,-1]   ##front left, rear left, rear right, front right
	return_code, wheel_joints[0]=sim.simxGetObjectHandle(client_id, 'rollingJoint_fl',sim.simx_opmode_blocking)
	return_code,wheel_joints[1]=sim.simxGetObjectHandle(client_id,'rollingJoint_rl',sim.simx_opmode_blocking)
	return_code,wheel_joints[2]=sim.simxGetObjectHandle(client_id,'rollingJoint_rr',sim.simx_opmode_blocking)
	return_code,wheel_joints[3]=sim.simxGetObjectHandle(client_id,'rollingJoint_fr',sim.simx_opmode_blocking)
	return wheel_joints


def encoders(client_id):
	return_code,signal_value=sim.simxGetStringSignal(client_id,'combined_joint_position',sim.simx_opmode_blocking)
	signal_value = signal_value.decode()
	joints_position = signal_value.split("%")

	for index,joint_val in enumerate(joints_position):
		joints_position[index]=float(joint_val)

	return joints_position


def nav_logic(client_id,t):
	i = 0
	j=0
	c=0
	n=1.3
	while(i<len(t)-1):
		vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
		transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
		detect = detect_qr_codes(transformed_image)
        #print(encoders(client_id))
		a = t[i+1][1]-t[i][1]
		b = t[i+1][0]-t[i][0]
		d = math.sqrt((a*a) + (b*b))
		if str(tuple(t[i+1])) in detect:
			a=0
			b=0
			i=i+1
		set_bot_movement(client_id, init_setup(client_id), -a*n,-b*n,0)
			
		


def shortest_path(target_points):
	N = len(target_points)
	M = 2
	t = [ [ 0 for i in range(M) ] for j in range(N+1) ]
	t[0] = [0,1]
	for i in range(N):
		t[i+1][0] = target_points[i][0]
		t[i+1][1] = target_points[i][1]+1
	for i in range(N):
		if t[i][0]>=8:
			t[i][0] = 8
		if t[i][1]>=11:
			t[i][1] = 11
	return t

	


def task_3_primary(client_id,target_points):
	set_bot_movement(client_id, init_setup(client_id), 0,0,0)
	nav_logic(client_id, shortest_path(target_points))

if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(2,3),(3,6),(11,11),(0,0)]    # You can give any number of different co-ordinates


	##################################################
	## NOTE: You are NOT allowed to make any changes in the code below ##

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

		task_3_primary(client_id, target_points)
		#set_bot_movement(client_id, init_setup(client_id), 0 , 0 , 0)
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
		print('\n[ERROR] Your task_3_primary function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()