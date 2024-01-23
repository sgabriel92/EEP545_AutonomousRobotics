#!/usr/bin/env python
import sys
import csv
import rospy 
import Utils as u
import numpy as np

sys.path.append("/home/robot/mushr_ws/src/assignment5/src/lineFollower")

from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from ackermann_msgs.msg import AckermannDriveStamped

from line_follower import LineFollower

try:
    xrange
except NameError:
    xrange = range

try:
    raw_input
except NameError:
    raw_input = input


#global
inferred_pose = [0.0,0.0,0.0]
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'

global mega_plan

def inferred_pose_cb(pose):
	global inferred_pose
	inferred_pose[0] = pose.pose.position.x
	inferred_pose[1] = pose.pose.position.y
	inferred_pose[2] = u.quaternion_to_angle(pose.pose.orientation)

def publish_plan(plan):
    pa = PoseArray()
    pa.header.frame_id = "/map"
    for i in xrange(len(plan)):
      config = plan[i]
      pose = Pose()
      pose.position.x = config[0]
      pose.position.y = config[1]
      pose.position.z = 0.0
      pose.orientation = u.angle_to_quaternion(config[2])
      pa.poses.append(pose)
    return pa 

if __name__ == '__main__':
	rospy.init_node("final_race", anonymous=True) # Initialize the node
	
	mega_plan = []
	plan_loc = '/home/robot/mushr_ws/src/assignment5/src/planner/plan.txt'

	with open(plan_loc, 'r') as f:
		reader = csv.reader(f)
		for plan in reader:
			temp_plan = []
			for p in plan:
				p = p.split(', ')
				p[:] = [float(p[0][1:]), float(p[1][:]), float(p[2][:len(p[2])-1])]
				temp_plan.append(p)
			mega_plan.append(temp_plan)

	pose_topic = '/pf/viz/inferred_pose' # Default val: '/car/pose'
	plan_lookahead = 2 # Starting val: 5
	translation_weight = 0.75 # Starting val: 1.0
	rotation_weight = 0.25 # Starting val: 0.0
	kp = 1 # Starting val: 1.0
	ki = 0.05 # -0.052 # Starting val: 0.0
	kd = 0.3  #-0.713 # Starting val: 0.0
	error_buff_length = 10 # Starting val: 10
	speed = 1.0 # Default val: 1.0

	plan_topic = '/planner_node/car_plan' # Default val: '/planner_node/car_plan'

	plan_pub = rospy.Publisher(plan_topic, PoseArray, queue_size=1)
	pf_pose_sub = rospy.Subscriber("/pf/viz/inferred_pose", PoseStamped, inferred_pose_cb)
	cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size = 10)

	raw_input("[Final Race] Press Enter to win the race...")  # Waits for ENTER key press
	
	print("[Final Race] length of mega-plan: ", len(mega_plan))

	for i in range(len(mega_plan)):
		print("[Final Race] Executing Plan segment: ", i+1)
		print(mega_plan[i])
		plan_pub.publish(publish_plan(mega_plan[i]))

		if i == 0:
			while inferred_pose[0] < 51.6:
				ads = AckermannDriveStamped()
				ads.header.frame_id = '/map'
				ads.header.stamp = rospy.Time.now()
				ads.drive.steering_angle = 0.0
				ads.drive.speed = -1.0
				cmd_pub.publish(ads)
			rospy.sleep(0.5)
		else:
			speed = 1.25
			plan_lookahead = 2
			if i in {4, 5}:
				speed = 1.0
			elif i in {8}:
				speed = 1.5

			if i in {6}:
				plan_lookahead = 3

			lf = LineFollower(mega_plan[i], pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, error_buff_length, speed)
			while not lf.completed:
				rospy.sleep(0.3)
			del lf

	print("[Final Race] Race completed...")
	rospy.spin() #dont kill






