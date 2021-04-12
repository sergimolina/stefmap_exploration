#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PointStamped,PoseStamped, Point, Vector3
import tf.transformations
from nav_msgs.msg import OccupancyGrid
import numpy as np
import json
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from stefmap_ros.srv import GetVisibilityMap
import rospkg
import os
from visualization_msgs.msg import Marker
from std_msgs.msg import String , Int16
import random
from orunav_msgs.msg import RobotReport
import math

class stefmap_exploration_node(object):

	def __init__(self):
		#parameters
		self.locations_file = rospy.get_param('~locations_file',"locations_coordinates.json")
		self.locations_frame_id = rospy.get_param('~locations_frame_id',"world")
		
		#ini variables
		self.active_robots = {}
		self.mission_status = {}
		self.entropy_map_received = False
		self.visibility_map_received = False
		self.exploration_allowed = 0
		self.robots_locations = {"robot1":{},"robot2":{},"robot3":{},"robot4":{},"robot5":{},"robot6":{},"robot7":{},"robot8":{},"robot9":{}}

		# read the info from the file
		rospy.loginfo("Read locations info from file")
		with open(self.locations_file,"r") as loc_file:
			self.locations_data = json.load(loc_file)
		rospy.loginfo("num of locations: "+str(len(self.locations_data["locations"])))

		#create the new file storing the info regardaing the visible cells for each location in stefmap_exploration/data/
		self.check_data_in_info_file()

		# subscribe to topics
		rospy.Subscriber("/entropy_map", OccupancyGrid, self.entropy_map_callback,queue_size=1)
		rospy.Subscriber("/active_robot_status",String, self.active_robot_status_callback,queue_size=1)
		rospy.Subscriber("/mission_status",String, self.mission_status_callback,queue_size=1)

		rospy.Subscriber("/exploration_allowed",Int16,self.exploration_allowed_callback,queue_size=1)

		rospy.Subscriber("/robot1/visibility_map",OccupancyGrid, self.visibility_map_robot1_callback,queue_size=1)
		rospy.Subscriber("/robot2/visibility_map",OccupancyGrid, self.visibility_map_robot2_callback,queue_size=1)
		rospy.Subscriber("/robot3/visibility_map",OccupancyGrid, self.visibility_map_robot3_callback,queue_size=1)
		rospy.Subscriber("/robot4/visibility_map",OccupancyGrid, self.visibility_map_robot4_callback,queue_size=1)
		rospy.Subscriber("/robot5/visibility_map",OccupancyGrid, self.visibility_map_robot5_callback,queue_size=1)
		rospy.Subscriber("/robot6/visibility_map",OccupancyGrid, self.visibility_map_robot6_callback,queue_size=1)
		rospy.Subscriber("/robot7/visibility_map",OccupancyGrid, self.visibility_map_robot7_callback,queue_size=1)
		rospy.Subscriber("/robot8/visibility_map",OccupancyGrid, self.visibility_map_robot8_callback,queue_size=1)
		rospy.Subscriber("/robot9/visibility_map",OccupancyGrid, self.visibility_map_robot9_callback,queue_size=1)

		rospy.Subscriber("/robot1/control/report",RobotReport, self.robot1_status_callback,queue_size=1)
		rospy.Subscriber("/robot2/control/report",RobotReport, self.robot2_status_callback,queue_size=1)
		rospy.Subscriber("/robot3/control/report",RobotReport, self.robot3_status_callback,queue_size=1)
		rospy.Subscriber("/robot4/control/report",RobotReport, self.robot4_status_callback,queue_size=1)
		rospy.Subscriber("/robot5/control/report",RobotReport, self.robot5_status_callback,queue_size=1)
		rospy.Subscriber("/robot6/control/report",RobotReport, self.robot6_status_callback,queue_size=1)
		rospy.Subscriber("/robot7/control/report",RobotReport, self.robot7_status_callback,queue_size=1)
		rospy.Subscriber("/robot8/control/report",RobotReport, self.robot8_status_callback,queue_size=1)
		rospy.Subscriber("/robot9/control/report",RobotReport, self.robot9_status_callback,queue_size=1)

		
		# create topic publishers
		self.exploration_goal_publisher = rospy.Publisher('/exploration_goal', PoseStamped, queue_size=1)
		self.markerPub = rospy.Publisher('location_markers', Marker,queue_size=10)	

		# start a timer to check regularly if we can send a exploration goal
		self.check_robot_availability_timer = rospy.Timer(rospy.Duration(10),self.check_robot_availability)
		self.update_locations_info_timer = rospy.Timer(rospy.Duration(5),self.update_locations_info)

		# show exploration location in rviz
		self.display_location_markers()

		self.run()

	def check_data_in_info_file(self):
		rospack = rospkg.RosPack()
		path_list = self.locations_file.split(os.sep)
		self.location_info_file = rospack.get_path('stefmap_exploration')+"/data/"+path_list[-1][:-5]+"_info.json"
		if os.path.isfile(self.location_info_file): 
			rospy.loginfo("Data file already exists, checking locations...")
			with open(self.location_info_file,"r") as info_file:
				self.info_file_locations_data = json.load(info_file)

			#check if the locations are the same or there are new ones
			for l in range(0,len(self.locations_data["locations"])):
				if l < len(self.info_file_locations_data["locations"]):
					if self.locations_data["locations"][l]["x"] != self.info_file_locations_data["locations"][l]["x"] or \
					   self.locations_data["locations"][l]["y"] != self.info_file_locations_data["locations"][l]["y"] or \
					   self.locations_data["locations"][l]["yaw"] != self.info_file_locations_data["locations"][l]["yaw"]:
						rospy.loginfo("Location "+str(l)+" is not the same, replacing and deleting old info")
						self.info_file_locations_data["locations"][l] = self.locations_data["locations"][l]
						self.info_file_locations_data["locations"][l]["num_visible_cells"] = 0
						self.info_file_locations_data["locations"][l]["visible_cells"] = []
					else:
						rospy.loginfo("Location "+str(l)+" is the same")
				else:
					rospy.loginfo("New location found, adding it to the list")
					self.info_file_locations_data["locations"].append(self.locations_data["locations"][l])
					self.info_file_locations_data["locations"][l]["num_visible_cells"] = 0
					self.info_file_locations_data["locations"][l]["visible_cells"] = []

			#delete extra loction saved 
			if len(self.info_file_locations_data["locations"]) > len(self.locations_data["locations"]):
				rospy.loginfo("Deleting extra locations saved")
				del self.info_file_locations_data["locations"][len(self.locations_data["locations"]):len(self.info_file_locations_data["locations"])] 

			with open(self.location_info_file,"w") as info_file:
				json.dump(self.info_file_locations_data,info_file,indent=4)

			self.locations_data = self.info_file_locations_data
		else: #make just a copy with the location info
			rospy.loginfo("Data file doesn't exists yet, creating the file.")
			for l in range(0,len(self.locations_data["locations"])):
				self.locations_data["locations"][l]["num_visible_cells"] = 0
				self.locations_data["locations"][l]["visible_cells"] = []

			with open(self.location_info_file,"w") as info_file:
				json.dump(self.locations_data,info_file,indent=4)	

	def update_locations_info(self,timer):
		# check the location of all robot
		if (len(self.active_robots) > 0):
			for robot in self.active_robots:
				robot_x = self.robots_locations[robot]["x"]
				robot_y = self.robots_locations[robot]["y"]

				# if robot is close to an exploration location update the visible cells
				for l in range(0,len(self.locations_data["locations"])):
					location_x = self.locations_data["locations"][l]["x"]
					location_y = self.locations_data["locations"][l]["y"]
					
					distance = math.sqrt((robot_x-location_x)*(robot_x-location_x)+(robot_y-location_y)*(robot_y-location_y))
					# distance under a theshold, update the visible cells from the location
					if distance <= 0.5:
						rospy.loginfo("---> Updating info location number:  "+str(l))
						if robot == "robot1":
							location_visibility_map = self.last_visibility_map_robot1
						elif robot == "robot2":
							location_visibility_map = self.last_visibility_map_robot2
						elif robot == "robot3":
							location_visibility_map = self.last_visibility_map_robot3
						elif robot == "robot4":
							location_visibility_map = self.last_visibility_map_robot4
						elif robot == "robot5":
							location_visibility_map = self.last_visibility_map_robot5
						elif robot == "robot6":
							location_visibility_map = self.last_visibility_map_robot6
						elif robot == "robot7":
							location_visibility_map = self.last_visibility_map_robot7
						elif robot == "robot8":
							location_visibility_map = self.last_visibility_map_robot8
						elif robot == "robot9":
							location_visibility_map = self.last_visibility_map_robot9

						num_visible_cells = 0
				 		visible_cells = []
				 		for c in range(0,len(location_visibility_map.data)):
				 			if location_visibility_map.data[c] == 100:
				 				num_visible_cells = num_visible_cells + 1
				 				visible_cells.append(c)

						self.locations_data["locations"][l]["num_visible_cells"] = num_visible_cells
						self.locations_data["locations"][l]["visible_cells"] = visible_cells

						with open(self.location_info_file,"w") as info_file:
							json.dump(self.info_file_locations_data,info_file,indent=4)

	def display_location_markers(self):
		for l in range(0,len(self.locations_data["locations"])):
			# put column
			location_marker = Marker()
			location_marker.header.frame_id = self.locations_frame_id
			location_marker.header.stamp = rospy.get_rostime()
			location_marker.type = 5 #line list
			location_marker.id = l
			location_marker.action = 0 # add
			location_marker.scale.x = 0.4
			location_marker.color.b = 1
			location_marker.color.a = 1

			location_point = Point()
			location_point.x = self.locations_data["locations"][l]["x"]
			location_point.y = self.locations_data["locations"][l]["y"]
			location_point.z = 0
			location_marker.points.append(location_point)

			location_point = Point()
			location_point.x = self.locations_data["locations"][l]["x"]
			location_point.y = self.locations_data["locations"][l]["y"]
			location_point.z = 5
			location_marker.points.append(location_point)

			self.markerPub.publish(location_marker)

			location_marker = Marker()
			location_marker.header.frame_id = self.locations_frame_id
			location_marker.header.stamp = rospy.get_rostime()
			location_marker.type = 9 #text
			location_marker.id = len(self.locations_data["locations"]) + l
			location_marker.action = 0 # add
			location_marker.scale.z = 1
			location_marker.color.b = 1
			location_marker.color.a = 1
			location_marker.pose.position.x = self.locations_data["locations"][l]["x"]
			location_marker.pose.position.y = self.locations_data["locations"][l]["y"]
			location_marker.pose.position.z = 5.5
			location_marker.text = "LOC "+str(l)

			self.markerPub.publish(location_marker)

	def display_chosen_location(self):
		location_marker = Marker()
		location_marker.header.frame_id = self.locations_frame_id
		location_marker.header.stamp = rospy.get_rostime()
		location_marker.type = 5 #line list
		location_marker.id = self.location_chosen
		location_marker.action = 0 # add
		location_marker.scale.x = 0.4
		location_marker.color.g = 1
		location_marker.color.a = 1

		location_point = Point()
		location_point.x = self.locations_data["locations"][self.location_chosen]["x"]
		location_point.y = self.locations_data["locations"][self.location_chosen]["y"]
		location_point.z = 0
		location_marker.points.append(location_point)

		location_point = Point()
		location_point.x = self.locations_data["locations"][self.location_chosen]["x"]
		location_point.y = self.locations_data["locations"][self.location_chosen]["y"]
		location_point.z = 5
		location_marker.points.append(location_point)

		self.markerPub.publish(location_marker)

		location_marker = Marker()
		location_marker.header.frame_id = self.locations_frame_id
		location_marker.header.stamp = rospy.get_rostime()
		location_marker.type = 9 #text
		location_marker.id = len(self.locations_data["locations"]) + self.location_chosen
		location_marker.action = 0 # add
		location_marker.scale.z = 1
		location_marker.color.g = 1
		location_marker.color.a = 1
		location_marker.pose.position.x = self.locations_data["locations"][self.location_chosen]["x"]
		location_marker.pose.position.y = self.locations_data["locations"][self.location_chosen]["y"]
		location_marker.pose.position.z = 5.5
		location_marker.text = "LOC "+str(self.location_chosen)

		self.markerPub.publish(location_marker)

	def entropy_map_callback(self,entropy_map_msg):
		self.last_entropy_map = entropy_map_msg
		self.entropy_map_received = True

	def visibility_map_robot1_callback(self,visibility_map_msg):
		self.last_visibility_map_robot1 = visibility_map_msg
		self.visibility_map_received = True

	def visibility_map_robot2_callback(self,visibility_map_msg):
		self.last_visibility_map_robot2 = visibility_map_msg
		self.visibility_map_received = True

	def visibility_map_robot3_callback(self,visibility_map_msg):
		self.last_visibility_map_robot3 = visibility_map_msg
		self.visibility_map_received = True

	def visibility_map_robot4_callback(self,visibility_map_msg):
		self.last_visibility_map_robot4 = visibility_map_msg
		self.visibility_map_received = True

	def visibility_map_robot5_callback(self,visibility_map_msg):
		self.last_visibility_map_robot5 = visibility_map_msg
		self.visibility_map_received = True

	def visibility_map_robot6_callback(self,visibility_map_msg):
		self.last_visibility_map_robot6 = visibility_map_msg
		self.visibility_map_received = True

	def visibility_map_robot7_callback(self,visibility_map_msg):
		self.last_visibility_map_robot7 = visibility_map_msg
		self.visibility_map_received = True

	def visibility_map_robot8_callback(self,visibility_map_msg):
		self.last_visibility_map_robot8 = visibility_map_msg
		self.visibility_map_received = True

	def visibility_map_robot9_callback(self,visibility_map_msg):
		self.last_visibility_map_robot9 = visibility_map_msg
		self.visibility_map_received = True

	def robot1_status_callback(self,robot_report_msg):
		self.robots_locations['robot1']['x'] = robot_report_msg.state.pose.position.x
		self.robots_locations['robot1']['y'] = robot_report_msg.state.pose.position.y

	def robot2_status_callback(self,robot_report_msg):
		self.robots_locations['robot2']['x'] = robot_report_msg.state.pose.position.x
		self.robots_locations['robot2']['y'] = robot_report_msg.state.pose.position.y

	def robot3_status_callback(self,robot_report_msg):
		self.robots_locations['robot3']['x'] = robot_report_msg.state.pose.position.x
		self.robots_locations['robot3']['y'] = robot_report_msg.state.pose.position.y

	def robot4_status_callback(self,robot_report_msg):
		self.robots_locations['robot4']['x'] = robot_report_msg.state.pose.position.x
		self.robots_locations['robot4']['y'] = robot_report_msg.state.pose.position.y

	def robot5_status_callback(self,robot_report_msg):
		self.robots_locations['robot5']['x'] = robot_report_msg.state.pose.position.x
		self.robots_locations['robot5']['y'] = robot_report_msg.state.pose.position.y

	def robot6_status_callback(self,robot_report_msg):
		self.robots_locations['robot6']['x'] = robot_report_msg.state.pose.position.x
		self.robots_locations['robot6']['y'] = robot_report_msg.state.pose.position.y

	def robot7_status_callback(self,robot_report_msg):
		self.robots_locations['robot7']['x'] = robot_report_msg.state.pose.position.x
		self.robots_locations['robot7']['y'] = robot_report_msg.state.pose.position.y

	def robot8_status_callback(self,robot_report_msg):
		self.robots_locations['robot8']['x'] = robot_report_msg.state.pose.position.x
		self.robots_locations['robot8']['y'] = robot_report_msg.state.pose.position.y

	def robot9_status_callback(self,robot_report_msg):
		self.robots_locations['robot9']['x'] = robot_report_msg.state.pose.position.x
		self.robots_locations['robot9']['y'] = robot_report_msg.state.pose.position.y

	def active_robot_status_callback(self,active_robots_status_msg):
		self.active_robots = json.loads(active_robots_status_msg.data)

	def mission_status_callback(self,mission_status_msg):
		self.mission_status = json.loads(mission_status_msg.data)

	def exploration_allowed_callback(self,exploration_allowed_msg):
		self.exploration_allowed = exploration_allowed_msg.data

	def check_robot_availability(self,timer):
		robots_free = 0
		rospy.loginfo("-----------------")
		rospy.loginfo("Checking exploration availability:")
		self.display_location_markers()

		if self.exploration_allowed:

			if (len(self.active_robots) > 0):
				already_exploring = 0
				# check if any robot is already exploring
				for robot in self.active_robots:
					if self.active_robots[robot]["operation"] == "EXPLORING":
						already_exploring = 1
						rospy.loginfo("Robot already exploring")
						self.display_chosen_location()
						break


				if already_exploring == 0:
					# check how many robots are free
					for robot in self.active_robots:
						if self.active_robots[robot]["status"] == "FREE":
							robots_free = robots_free + 1

					if robots_free > 0:
						if (self.mission_status["seconds_left_next"] == "-" or self.mission_status["seconds_left_next"] > 300):
							for robot in self.active_robots:
								if self.active_robots[robot]["status"] == "FREE":
									self.choose_location_to_explore()
									#self.display_location_markers()
									self.display_chosen_location()
									
									rospy.loginfo("Requesting "+str(robot)+" to explore location "+str(self.location_chosen))
									exploration_goal_msg = PoseStamped()
									exploration_goal_msg.header.stamp = rospy.get_rostime()
									exploration_goal_msg.header.frame_id = self.locations_frame_id
									exploration_goal_msg.pose.position.x = self.locations_data["locations"][self.location_chosen]["x"]
									exploration_goal_msg.pose.position.y =self.locations_data["locations"][self.location_chosen]["y"]

									qx,qy,qz,qw = tf.transformations.quaternion_from_euler(0,0,self.locations_data["locations"][self.location_chosen]["yaw"])
									exploration_goal_msg.pose.orientation.x = qx
									exploration_goal_msg.pose.orientation.y = qy
									exploration_goal_msg.pose.orientation.z = qz
									exploration_goal_msg.pose.orientation.w = qw

									self.exploration_goal_publisher.publish(exploration_goal_msg)
									break

						elif (self.mission_status["seconds_left_next"] < 300 and robots_free > 1):
							for robot in self.active_robots:
								if self.active_robots[robot]["status"] == "FREE":
									self.choose_location_to_explore()
									#self.display_location_markers()
									self.display_chosen_location()
									
									rospy.loginfo("Requesting "+str(robot+" to explore location "+str(self.location_chosen)))
									exploration_goal_msg = PoseStamped()
									exploration_goal_msg.header.stamp = rospy.get_rostime()
									exploration_goal_msg.header.frame_id = self.locations_frame_id
									exploration_goal_msg.pose.position.x = self.locations_data["locations"][self.location_chosen]["x"]
									exploration_goal_msg.pose.position.y =self.locations_data["locations"][self.location_chosen]["y"]

									qx,qy,qz,qw = tf.transformations.quaternion_from_euler(0,0,self.locations_data["locations"][self.location_chosen]["yaw"])
									exploration_goal_msg.pose.orientation.x = qx
									exploration_goal_msg.pose.orientation.y = qy
									exploration_goal_msg.pose.orientation.z = qz
									exploration_goal_msg.pose.orientation.w = qw

									self.exploration_goal_publisher.publish(exploration_goal_msg)
									break
						else:
							rospy.loginfo( "Only 1 free robot and the next mission is too close in time")

					else:
						rospy.loginfo( "No free robots")

			else:
				rospy.loginfo( "No active robots")
		else:
			rospy.loginfo("Exploration not allowed")


	def choose_location_to_explore(self):
		# before checking the entropies, give priority to unvisited locations
		for l in range(0,len(self.locations_data["locations"])):
			if self.info_file_locations_data["locations"][l]["num_visible_cells"] == 0:
				rospy.loginfo("Location chosen for being unvisited: "+str(l))
				self.location_chosen = l
				return


		if self.entropy_map_received:
			# 1 - compute the entropy for each location
			location_entropies = []
			location_visible_cells = []
			location_avg_entropy_cell = []
			for l in range(0,len(self.locations_data["locations"])):
				total_location_entropy = 0
				for c in range(0,self.locations_data["locations"][l]["num_visible_cells"]):
					cell = self.locations_data["locations"][l]["visible_cells"][c]
					total_location_entropy = total_location_entropy + self.last_entropy_map.data[cell]
				location_entropies.append(total_location_entropy)
				location_visible_cells.append(self.locations_data["locations"][l]["num_visible_cells"])
				location_avg_entropy_cell.append(float(total_location_entropy)/float(self.locations_data["locations"][l]["num_visible_cells"]))

			rospy.loginfo("location entropies: "+str(location_entropies))
			rospy.loginfo("location visible cells: "+str(location_visible_cells))
			rospy.loginfo("avg entropy/cell: "+ str(location_avg_entropy_cell))
			
			# 2 - compute the probability of each location being visited
			total_map_entropy = sum(self.last_entropy_map.data)
			total_locations_entropy = float(sum(location_entropies))

			location_probabilities = []
			for l in range(0,len(self.locations_data["locations"])):
				location_probabilities.append((1/(location_entropies[l]/total_locations_entropy)))
			 # normalisation step
			location_probabilities[:] = [x / sum(location_probabilities) for x in location_probabilities]
			rospy.loginfo("location probabilities: "+str(location_probabilities))

			# 3 - decide one location using the previous probabilities
			self.location_chosen = np.where(np.random.multinomial(1,location_probabilities))[0][0]
			rospy.loginfo("location chosen: "+str(self.location_chosen))
		
		else: # in case no entropy received all location have the same probability to be chosen
			self.location_chosen = random.randint(0,len(self.locations_data["locations"])-1)
			rospy.loginfo("location chosen: "+str(self.location_chosen))

		
	def run(self):
		while not rospy.is_shutdown():
			rospy.spin()

if __name__ == '__main__':
	rospy.init_node('stefmap_exploration_node', anonymous=True)
	sen = stefmap_exploration_node()
