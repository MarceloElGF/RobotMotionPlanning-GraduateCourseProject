import cv2
import numpy as np
import math
import random
from PIL import Image, ImageTk

from Path import *
# from Queue import Queue

class prm_node:
	def __init__(self,map_i=int(0),map_j=int(0)):
		self.map_i = map_i
		self.map_j = map_j
		self.edges = [] #edges of child nodes
		self.parent = None #parent node


class prm_edge:
	def __init__(self,node1=None,node2=None):
		self.node1 = node1 #parent node
		self.node2 = node2 #child node

#You may modify it to increase efficiency as list.append is slow
class prm_tree:
	def __init__(self):
		self.nodes = []
		self.edges = []

	def add_nodes(self,node):
		self.nodes.append(node)

	#add an edge to our PRM tree, node1 is parent, node2 is the kid
	def add_edges(self,node1,node2): 
		edge = prm_edge(node1,node2)
		self.edges.append(edge)
		node1.edges.append(edge)
		node2.parent=edge.node1


class path_planner:
	def __init__(self,graphics):
		self.graphics = graphics

		self.controller = self.graphics.environment.robots[0].controller ##### added according to webpage
		self.robot = self.graphics.environment.robots[0] ##### added according to webpage
		# self.graphics.scale = 400 #half pixel number on canvas, the map should be 800 x 800
		# self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2

		self.costmap = self.graphics.map
		self.map_width = self.costmap.map_width
		self.map_height = self.costmap.map_height

		self.pTree=prm_tree()

		self._init_path_img()
		self.path = Path()
		
		self.set_start(world_x = .0, world_y = .0)
		self.set_goal(world_x = 50.0, world_y = 0.0, world_theta = .0) # map 8: world_x = 0, world_y = -230.0, world_theta = .0 # world_x = -100.0, world_y = 200.0, world_theta = .0

		self.plan_path()
		self._show_path()

	def set_start(self, world_x = 0, world_y = 0, world_theta = 0): #world_theta = .0):
		self.start_state_map = Pose()
		map_i, map_j = self.world2map(world_x,world_y)
		print("Start with %d, %d on map"%(map_i,map_j))
		self.start_state_map.set_pose(map_i,map_j,world_theta)
		self.start_node = prm_node(map_i,map_j)
		self.pTree.add_nodes(self.start_node)

	def set_goal(self, world_x, world_y, world_theta = 0):
		self.goal_state_map = Pose()
		goal_i, goal_j = self.world2map(world_x, world_y)
		print ("goal is %d, %d on map"%(goal_i, goal_j))
		self.goal_state_map.set_pose(goal_i, goal_j, world_theta)
		self.goal_node = prm_node(goal_i,goal_j)
		self.pTree.add_nodes(self.goal_node)

	#convert a point a map to the actual world position
	def map2world(self,map_i,map_j):
		world_x = -self.graphics.environment.width/2*self.graphics.scale + map_j
		world_y = self.graphics.environment.height/2*self.graphics.scale - map_i
		return world_x, world_y

	#convert a point in world coordinate to map pixel
	def world2map(self,world_x,world_y):
		map_i = int(self.graphics.environment.width/2*self.graphics.scale - world_y)
		map_j = int(self.graphics.environment.height/2*self.graphics.scale + world_x)
		if(map_i<0 or map_i>=self.map_width or map_j<0 or map_j>=self.map_height):
			warnings.warn("Pose %f, %f outside the current map limit"%(world_x,world_y))

		if(map_i<0):
			map_i=int(0)
		elif(map_i>=self.map_width):
			map_i=self.map_width-int(1)

		if(map_j<0):
			map_j=int(0)
		elif(map_j>=self.map_height):
			map_j=self.map_height-int(1)

		return map_i, map_j

	def _init_path_img(self):
		self.map_img_np = 255*np.ones((int(self.map_width),int(self.map_height),4),dtype = np.int16)
		# self.map_img_np[0:-1][0:-1][3] = 0
		self.map_img_np[:,:,3] = 0

	def _show_path(self):
		for pose in self.path.poses:
			map_i = pose.map_i
			map_j = pose.map_j 
			self.map_img_np[map_i][map_j][1] =0
			self.map_img_np[map_i][map_j][2] =0
			self.map_img_np[map_i][map_j][3] =255

		np.savetxt("file.txt", self.map_img_np[1])

		self.path_img=Image.frombytes('RGBA', (self.map_img_np.shape[1],self.map_img_np.shape[0]), self.map_img_np.astype('b').tostring())
		# self.path_img = toimage(self.map_img_np)
		#self.path_img.show()
		self.graphics.draw_path(self.path_img)

	def check_vicinity(self,x1,y1,x2,y2,threshold = 1.0):
		if(math.sqrt((x1-x2)**2+(y1-y2)**2)<threshold):
			return True
		else:
			return False

	def plan_path(self):
		self.path.clear_path()                                                # clears the previous path each time a new goat point is given
		node_list = np.array([[self.start_node.map_i,self.start_node.map_j]]) # Initializes the array that stores all the node points in "map" (Row #, Column #) coordinates 
		infy = 9999 										                  # sets the varible infy that represents infinity to 9999
		Array_1 = np.full_like(self.costmap.costmap, infy) 	                  # initializes Array 1 to be 9999 or infinity that is the same size of the cost array, This Array stores the value of the element
		num_of_rows = Array_1.shape[0]						                  # calculates the number of rows in Array_1
		num_of_columns = Array_1.shape[1]									  # calculates the number of columns in Array_1
		max_el_row = num_of_rows - 1										  # calculates the maximum row # in the Array (ie 0 = 1st row)
		max_el_col = num_of_columns - 1										  # calculates the maximum column # in the Array (ie 0 = 1st column)
		maximum_cost = int(np.max(self.costmap.costmap))    				  # calculates the maximum cost from the cost map which means pixel is a obstacle
		node_gen = 5														  # initializes the variable to start the while loop
		node_counter = 0													  # initializes the variable to count how many nodes were explored
		parent_node_tracker = np.array([[self.start_node.map_i,self.start_node.map_j]]) # stores the parent node for the expanded node
		
		while node_gen <= 20: 					 # while loop that continues until their is a path to the goal point
			r_col = random.randint(0,max_el_col) # random number for the column number
			r_row = random.randint(0,max_el_row) # random number for the row number
			node_counter = node_counter +1 		 # keeps track of how many nodes were expanded

			for nodess in node_list: 									   # for loop that runs through each node in node_ list
				points_check = bresenham(nodess[0],nodess[1],r_row,r_col)  # variable that stores all the points from the node being checked from node list to the random node picked
				hit_obstacle = False 									   # initialization of varible that states if a obstacle was hit. Initialized at False because we assume it works until proven otherwise

				for p in points_check: 									   # for the points in points_check
					if (self.costmap.costmap[p[0]][p[1]]) == maximum_cost: # if the point in the costmap is a maximum value AKA"obstacle"
						hit_obstacle = True								   # then a obstacle was hit
						break
				
				if hit_obstacle == False:								      							# if random node to node in node list dont hit an obstacle
					node_list = np.vstack([node_list, [r_row, r_col]])									# add the random node to the node list
					parent_node_tracker = np.vstack([parent_node_tracker,[nodess[0], nodess[1]]])		# add the parent node to the parent node list
					points_check_end = bresenham(r_row,r_col,self.goal_node.map_i,self.goal_node.map_j) # points from random node to the end goal
					hit_obstacle_end_goal = False                                                       # initialization of varible that states if a obstacle was hit. Initialized at False because we assume it works until proven otherwise

					for pp in points_check_end:                                                         # for the points in points_check_end
						if(self.costmap.costmap[pp[0]][pp[1]]) == maximum_cost:							# if the point in the costmap is a maximum value AKA"obstacle"
							hit_obstacle_end_goal = True												# then a obstacle was hit
							break 																		# break from the for loop since other points dont need to be check if we already hit

					if hit_obstacle_end_goal == False:													# if random node to node in node list dont hit an obstacle
						node_gen = 100																	# stop the while loop generating nodes
						node_list = np.vstack([node_list, [self.goal_node.map_i,self.goal_node.map_j]]) # add the goal node to the node list
						parent_node_tracker = np.vstack([parent_node_tracker,[r_row,r_col]])			# add the parent goal node to the node list
						break

					if hit_obstacle_end_goal == True:													# if the node can't see the end goal due to obstacle
						break																			# break from the for loop

		length_node_list = len(node_list) 											# calculates the number of element in the array holding the nodes
		i = range(length_node_list) 												# creates a range for the number of element in the array holding the nodes
		nodes_plot = np.array([node_list[-1]]) 										# initializes the nodes_plot array by starting at the start pt

		goal_reached = False 														# goal has not been reached
		while not goal_reached: 													# While loop until the goal has been reached
			for index_number in i:													# running through the nodes to plot
				if (nodes_plot[0] == node_list[0]).all():							# if the node plot is equal to the start
					goal_reached = True												# goal has been reached and can brak from for loop
					break														    # break from for loop

				if (node_list[index_number] == nodes_plot[0]).all():					   # if both the node list index is the same as the node to plot
					nodes_plot = np.vstack([parent_node_tracker[index_number],nodes_plot]) # add the parent node to the node to plot list
					break																   # break from for loop

		pt_plot = np.empty((0, 2), dtype=int)				        # initializes the pt_plot array
		length_nodes_plot = len(nodes_plot) 						# calculates the number of element in the array holding the nodes to plot
		ii = range(length_nodes_plot)								# creates a range for the number of element in the array holding the nodes
		for index_number in ii:										# for loop to plot the path between the nodes
			if index_number == length_nodes_plot-1:					# if the index number is the last one we are done since the goal is reached and do not need to make an additional path
				break												# break from for loop
			
			points = bresenham(nodes_plot[index_number][0],nodes_plot[index_number][1],nodes_plot[index_number+1][0],nodes_plot[index_number+1][1]) # calculate the points between the nodes
			for p in points:
				self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0)) # plot the points on map
				pt_plot = np.vstack([pt_plot,p]) 						# adds the plotted point to the vector storing the points that were plotted

		numb_disc = 100 											    # number of points between destination points
		pt_start = 75   												# number of point between the 1st point and 1st destination point
		disc_points = pt_plot[pt_start::numb_disc] 						# stores all the destination points
		disc_points = np.vstack([disc_points, pt_plot[-1]]) 			# adds the last point of the pt_plot to the destintion points
		length_disc_points = len(disc_points) 							# gets the length for the vector storing each p controller navigation point
		iii = range(length_disc_points)       							# creates the index range
		v_base = [1, 0] 												# reference line for angle
		v1 = np.array(v_base) 											# reference line for angle in numpy array

		for index_number in iii:
			if index_number == length_disc_points-1:					# if the index number is the last one we are done since the goal is reached and do not need to make an additional path
				self.robot.state_des.add_destination(disc_points[index_number][1]-250, -1*disc_points[index_number][0]+250, angle) # add a destination point
				break													# break from for loop
			
			v2_x = disc_points[index_number+1][1] - disc_points[index_number][1]   # creates the x component of the vector
			v2_y = disc_points[index_number+1][0] - disc_points[index_number][0]   # creates the y component of the vector
			v2 = [v2_x,v2_y]													   # creates the vector
			dot = np.dot(v1,v2) 												   # dot product between the 2 vectors
			mag = np.linalg.norm(v1) * np.linalg.norm(v2)						   # normalizes each vector and then gets the magnitude
			cos_angle = np.clip(dot / mag, -1.0, 1.0)							   # calculates the cos angle
			angle = np.arccos(cos_angle)										   # calculates the angle between the 2 vectors

			if disc_points[index_number+1][0] > disc_points[index_number][0]:      # if the new destination point is going downward
				angle = angle *-1												   # make the angle go downward

			self.robot.state_des.add_destination(disc_points[index_number][1]-250, -1*disc_points[index_number][0]+250, angle) # add a destination point


# bresenham algorithm for line generation on grid map
# from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
def bresenham(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep: # if the slope is steeper, we swap the x and ys to help with Bresenhams formula
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()

    # print points
    return points
