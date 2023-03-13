#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import geometric_form_finding_mod, robotic_operation_mod, robotic_collaboration_mod
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import numpy as np
import tf
from gripper import rg6
import os
from webclick_initialize import fwebdriver
import socket

# import pakages

class main(object):
	def __init__(self):
		n = int(input('enter number of the unit: '))			
		d = float(input('Enter the depth of the components: ')
		l = float(input('Enter the length of the main components: ')
		xm =  float(input('Enter the distance of MiR to structure: ')
		xs = float(input('Enter the distance of MiR to storage: ')
		# input initial parameters for the robotic construction

		rospy.init_node()	# initialize the ROS node

		ycc,zcc,theta = geometric_form_finding_mod.component_info(n,d,l)	# calculate the position of components CC and orientation of components MC using geometric form finding module

		file = open("cc_apriltag_id.txt","r")		
		cc_id = file.read()
		file.close()

		file = open("mc_apriltag_id.txt","r")
		mc_id = file.read()
		file.close()
		# read the Apriltag id corresponding to CC-i and MC-i from files 

		for i in range(1,n):

			ym[i] = robotic_operation_mod.loc_mob(ycc[i])	# calculate ym,i of the mobile robot
			robotic_operation_mod.moving(xm, ym[i])		# move the mobile robot to the target location
			robotic_operation_mod.alignment()		# align the mobile robot to parallel with installed component MC-(i-1)
			ys[i] = robotic_operation_mod.loc_store(ycc[i])	# calculate ys,i of the storage area
 			robotic_operation_mod.targeting(cc_id[i],xs,ys[i])	 # targeting CC-i
			robotic_operation_mod.install_cc(theta[i-1],d)	# installing CC-i
			robotic_operation_mod.targeting(mc_id[i],ys[i]) # targeting MC-i
			robotic_operation_mod.install_mc(theta[i-1],d,l) # installing MC-i

	
			while (robotic_collaboration_mod.check_team_b()==0):
				print("waiting for Team B to complete the installation ...")
			else:
				print("going to next construction stage ...")
			end
			# waiting for robotic Team B if Team B has not yet installed MC-i on another side of the structure.

if __name__ == "__main__":
    operation = main()



