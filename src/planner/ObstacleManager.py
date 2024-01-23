import cv2
import math
import numpy
import Utils
from nav_msgs.srv import GetMap
import rospy

class ObstacleManager(object):

	def __init__(self, mapMsg, car_width, car_length, collision_delta):
		self.map_info = mapMsg.info
		self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape(
			(mapMsg.info.height, mapMsg.info.width, 1))

		# Retrieve the map dimensions
		height, width, channels = self.mapImageGS.shape
		self.mapHeight = height
		self.mapWidth = width
		self.mapChannels = channels

		# Binarize the Image
		self.mapImageBW = 255 * numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
		self.mapImageBW[self.mapImageGS == 0] = 0

		# Obtain the car length and width in pixels
		self.robotWidth = int(car_width + 1 / self.map_info.resolution + 0.5)
		self.robotLength = int(car_length + 1 / self.map_info.resolution + 0.5)
		self.collision_delta = collision_delta

		badPoints = [[2280,self.mapHeight-800],[1685,self.mapHeight-970],[1390,self.mapHeight-910],[680,self.mapHeight-420],[725,self.mapHeight-635]]
		radius = 35
		for bbp in badPoints:
			#print("Inserting bad boi points...!")
			if bbp[0]==725:
				radius = 60
			else:
				radius = 35
			for x in range(bbp[0]-radius,bbp[0]+radius):
				for y in range(bbp[1]-radius,bbp[1]+radius):
#					print("hehe")
					self.mapImageBW[1235-y][x] = 255

	# Check if the passed config is in collision
	# config: The configuration to check (in meters and radians)
	# Returns False if in collision, True if not in collision
	def get_state_validity(self, config):

		# Convert the configuration to map-coordinates -> mapConfig is in pixel-space
		mapConfig = Utils.world_to_map(config, self.map_info)

		# ---------------------------------------------------------
		# YOUR CODE HERE
		#
		# Return true or false based on whether the robot's configuration is in collision
		# Use a square to represent the robot, return true only when all points within
		# the square are collision free
		#
		# Also return false if the robot is out of bounds of the map
		#
		# Although our configuration includes rotation, assume that the
		# square representing the robot is always aligned with the coordinate axes of the
		# map for simplicity
		# ----------------------------------------------------------
		#BoundaryCheck
		if mapConfig[0] >= self.mapWidth or mapConfig[1] >= self.mapHeight:
			return False
		
		i, j = mapConfig[1], mapConfig[0]

		if self.mapImageBW[i][j] == 255:
			return False

		x1, x2 = int(mapConfig[0]) - self.robotLength // 2, int(mapConfig[0]) + self.robotLength // 2
		# y1, y2 = int(mapConfig[1]) - self.robotLength / 2, int(mapConfig[1]) + self.robotLength / 2
		y1, y2 = int(mapConfig[1]) + self.robotLength // 2, int(mapConfig[1]) - self.robotLength // 2

		# print('mapConfig', mapConfig, 'x1, x2', (x1, x2), 'y1, y2', (y1, y2))
		for i in range(x1, x2 + 1):
			for j in range(y1, y2 + 1):
				if self.mapImageBW[i][j] == 255:
					return False
		

		return True

	# Discretize the path into N configurations, where N = path_length / self.collision_delta
	#
	# input: an edge represented by the start and end configurations
	#
	# return three variables:
	# list_x - a list of x values of all intermediate points in the path
	# list_y - a list of y values of all intermediate points in the path
	# edgeLength - The euclidean distance between config1 and config2
	def discretize_edge(self, config1, config2):
		list_x, list_y = [], []
		edgeLength = 0
		# -----------------------------------------------------------
		# YOUR CODE HERE
		# -----------------------------------------------------------
		edgeLength += numpy.sqrt(
					(config1[0] - config2[0]) ** 2 + (config1[1] - config2[1]) ** 2
				)

		n = int(edgeLength / self.collision_delta)
		list_x = numpy.linspace(config1[0], config2[0], n)
		list_y = numpy.linspace(config1[1], config2[1], n)

		return list_x, list_y, edgeLength


	# Check if there is an unobstructed edge between the passed configs
	# config1, config2: The configurations to check (in meters and radians)
	# Returns false if obstructed edge, True otherwise
	def get_edge_validity(self, config1, config2):
		# -----------------------------------------------------------
		# YOUR CODE HERE
		#
		# Check if endpoints are obstructed, if either is, return false
		# Find path between two configs by connecting them with a straight line
		# Discretize the path with the discretized_edge function above
		# Check if all configurations along path are obstructed
		# -----------------------------------------------------------
		if not self.get_state_validity(config1) or not self.get_state_validity(config2):
			return False

		list_x, list_y, _ = self.discretize_edge(config1, config2)
		for x, y in zip(list_x, list_y):
			if not self.get_state_validity([x, y]):
				return False
		return True


# Write Your Test Code For Debugging
if __name__ == '__main__':
	rospy.init_node("obstacle_manager")
	map_service_name = rospy.get_param("~static_map", "static_map")
	print("Getting map from service: ", map_service_name)
	rospy.wait_for_service(map_service_name)

	graph_file = rospy.get_param("~graph_file", None)
	map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

	car_width, car_length = 0.16, 0.33
	collision_delta = 1

	om = ObstacleManager(map_msg, car_width, car_length, collision_delta)
	config1 = [-10, 3]
	config2 = [-1, 2]
	config3 = [-10, 4]

	print(om.get_edge_validity(config1, config2))
