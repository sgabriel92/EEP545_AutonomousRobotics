#!/usr/bin/env python

import sys
import rospy
import Utils as u
import numpy as np
import csv


sys.path.append("/home/robot/catkin_ws/src/assignment5/src/planner")

from geometry_msgs.msg import PoseArray
from nav_msgs.srv import GetMap
from assignment5.srv import *

try:
    xrange
except NameError:
    xrange = range

try:
    raw_input
except NameError:
    raw_input = input


PLANNER_SERVICE_TOPIC = '/planner_node/get_car_plan'

global mega_plan

def plan_cb(pose_array):
	print("[planner.py] Reading Plan Segment...")
	plan = [[]]*len(pose_array.poses)
	i = 0 
	for pose in pose_array.poses:
		plan[i] = [pose.position.x,pose.position.y,u.quaternion_to_angle(pose.orientation)]
		i += 1
	print("[planner.py] Length of plan-seg: ", len(plan))
	mega_plan.append(plan)



if __name__ == '__main__':
	
	rospy.init_node("letsrace", anonymous=True) # Initialize the node

	plan_topic = '/planner_node/car_plan'
	map_service_name = rospy.get_param("~static_map", "static_map")
	print("[planner.py] Getting map from service: ", map_service_name)
	rospy.wait_for_service(map_service_name)
	map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info
	height = map_info.height

	rospy.wait_for_service(PLANNER_SERVICE_TOPIC)
	get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)

	plan_sub = rospy.Subscriber(plan_topic, PoseArray, plan_cb)

	mega_plan = []

	plan_loc = '/home/robot/catkin_ws/src/assignment5/src/planner/plan.txt'

	# goodPoints = [[2500, 640, 0.0],[2600, 660, 0.0],[1880, 440, -2.844],[1435, 545, 2.099],[1250, 460, 2.899],[1002,438,-2.619],[540, 835, -2.179]]
	#goodPoints = [[2500,640, 1.1], [2600,660, 1.1], [1880,440, -2.844], [1400,170,0.0], [1600, 330, 0.0], [1250,460, 2.899], [970, 450, -1.06], [540,835,-2.179]]
	goodPoints = [[2500,height-580, 0.0], [2600,height-575, 0.0], [2400,height-525, 0.0], [1800,height-715,0.0], [1845, height-825, 0.0], [1465,height-1160, 0.0], [1150, height-725, 0.0], [565,height-535,0.0], [600,height-400,0.0]]


	raw_input("[planner.py] Press Enter to plan...")  # Waits for ENTER key press
	
	for i in xrange(len(goodPoints) - 1):  
		print("[planner.py] Generating plan to point: ", i+1)
		try:
			resp = get_plan(u.map_to_world(goodPoints[i], map_info), u.map_to_world(goodPoints[i+1], map_info)) 
			rospy.sleep(0.1)
		except rospy.ServiceException as e:
			print ('[planner.py] done planning - but error on return')

	for plan in mega_plan:
		for i in range(1,len(plan)):
			plan[i][2] = np.arctan2((plan[i][1]-plan[i-1][1]),(plan[i][0]-plan[i-1][0]))

	with open(plan_loc, 'w') as f:
		writer = csv.writer(f)
		writer.writerows(mega_plan)





