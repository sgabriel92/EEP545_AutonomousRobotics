import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan
  def plan(self):
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id

    self.closed = {} # The closed list 
    self.parent = {self.sid:None} # A dictionary mapping children to their parents
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list
    self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
    self.planIndices = []
    self.cost = 0
    # ------------------------------------------------------------
    # YOUR CODE HERE
    # 
    # Implement A*
    # Functions that you will probably use
    # - self.get_solution()
    # - self.planningEnv.get_successors()
    # - self.planningEnv.get_distance()
    # - self.planningEnv.get_heuristic()
    # Note that each node in the graph has both an associated id and configuration
    # You should be searching over ids, not configurations. get_successors() will return
    #   the ids of nodes that can be reached. Once you have a path plan
    #   of node ids, get_solution() will compute the actual path in SE(2) based off of
    #   the node ids that you have found.
    #-------------------------------------------------------------
    while self.open:
          cur_node_id = min(self.open, key=self.open.get)
          cur_node_val = self.open[cur_node_id]
          self.open.pop(cur_node_id)
          self.closed[cur_node_id] = cur_node_val

          # goal
          if cur_node_id == self.tid:
            plan = self.get_solution(cur_node_id)
            l = len(plan)
            print('Length of the pre-plan is ' + str(l) + ' poses')
            plan = self.post_process(plan, 5)
            l = len(plan)
            print('Length of the post-plan is ' + str(l) + ' poses')
            self.simulate(plan)
            print('------------------------------------')
            return plan

          # not goal
          child_ids = self.planningEnv.get_successors(cur_node_id)
          cur_config = self.planningEnv.get_config(cur_node_id)
          for child_id in child_ids:
            if child_id not in self.closed.keys():
              child_g = cur_node_val + self.planningEnv.get_distance(cur_node_id, child_id)
              child_h = self.planningEnv.get_heuristic(child_id, self.tid)
              child_f = child_g + child_h
              if child_id not in self.open.keys():
                if self.planningEnv.manager.get_edge_validity(cur_config, self.planningEnv.get_config(child_id)):
                  self.open[child_id] = child_f
                  self.parent[child_id] = cur_node_id
              elif child_g < self.open[child_id] - child_h:
                if self.planningEnv.manager.get_edge_validity(cur_config, self.planningEnv.get_config(child_id)):
                  self.open[child_id] = child_f
                  self.parent[child_id] = cur_node_id    

    return []

  # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
  def post_process(self, plan, timeout):

    t1 = time.time()
    elapsed = 0
    while elapsed < timeout: # Keep going until out of time
      # ---------------------------------------------------------
      # YOUR CODE HERE
      
      # Pseudocode
      
      # Pick random id i
      # Pick random id j
      # Redraw if i == j
      # Switch i and j if i > j
     
      # if we can find path between i and j (Hint: look inside ObstacleManager.py for a suitable function)
        # Get the path
        # Reformat the plan such that the new path is inserted and the old section of the path is removed between i and j
        # Be sure to CAREFULLY inspect the data formats of both the original plan and the plan returned
        # to ensure that you edit the path correctly
      i, j = 0, 0
      while i == j:
        i = numpy.random.choice(numpy.arange(len(plan)))
        j = numpy.random.choice(numpy.arange(len(plan)))
      if i > j:
        i, j = j, i
      if self.planningEnv.manager.get_edge_validity(plan[i], plan[j]):
        list_x, list_y, _ = self.planningEnv.manager.discretize_edge(plan[i], plan[j])
        if len(list_x) < (j-i+1):
          new_plan = []
          new_plan.extend(plan[:i])
          new_poses = [[list_x[k], list_y[k]] for k in range(len(list_x))]
          new_plan.extend(new_poses)
          if j < len(plan):
            new_plan.extend(plan[j+1:])
          plan = new_plan

      elapsed = time.time() - t1
    return plan

  # Backtrack across parents in order to recover path
  # vid: The id of the last node in the graph
  def get_solution(self, vid):

    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]

    plan = []
    planID.reverse()
    for i in range(len(planID) - 1):
      startConfig = self.planningEnv.get_config(planID[i])
      goalConfig = self.planningEnv.get_config(planID[i + 1])
      px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
      plan.append([list(a) for a in zip(px, py)])
      self.planIndices.append(len(plan))
      self.cost += clen

    flatPlan = [item for sublist in plan for item in sublist]
    return flatPlan

  # Visualize the plan
  def simulate(self, plan):
    # Get the map
    envMap = 255*(self.planningEnv.manager.mapImageBW+1) # Hacky way to get correct coloring
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    for i in range(numpy.shape(plan)[0]-1): # Draw lines between each configuration in the plan
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()
