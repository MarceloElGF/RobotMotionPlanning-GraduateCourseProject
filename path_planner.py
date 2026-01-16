import cv2
import numpy as np
import math
import warnings
from PIL import Image, ImageTk

from bsTree import *
from Path import *

import heapq
class path_planner:
	def __init__(self,graphics):
		self.graphics = graphics
		# self.graphics.scale = 400 #half pixel number on canvas, the map should be 800 x 800
		# self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2

		self.costmap = self.graphics.map
		self.map_width = self.costmap.map_width
		self.map_height = self.costmap.map_height

		self._init_path_img()
		self.path = Path()

	
		self.set_start(world_x = 0, world_y = 0)
		self.set_goal(world_x = 0, world_y = -175, world_theta = .0) #self.set_goal(world_x = 20.0, world_y = 20.0, world_theta = .0) self.set_goal(world_x = -100.0, world_y = 100.0, world_theta = .0)

		self.plan_path()
		self._show_path()


	def set_start(self, world_x = 0, world_y = 0, world_theta = 0):
		self.start_state_map = Pose()
		map_i, map_j = self.world2map(world_x,world_y)
		print("Start with %d, %d on map"%(map_i,map_j))
		self.start_state_map.set_pose(map_i,map_j,world_theta)


	def set_goal(self, world_x, world_y, world_theta = 0):
		self.goal_state_map = Pose()
		map_i, map_j = self.world2map(world_x, world_y)
		print ("our new goal is %d, %d on map"%(map_i,map_j))
		self.goal_state_map.set_pose(map_i, map_j, world_theta)


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

		self.path_img=Image.frombytes('RGBA', (self.map_img_np.shape[1],self.map_img_np.shape[0]), self.map_img_np.astype('b').tostring())
		self.graphics.draw_path(self.path_img)

		# If you want to save the path as an image, un-comment the following line:
		# self.path_img.save('Log\path_img.png')
		
		# If you want to output an image of map and path, un-comment the following two lines
		# self.path_img = toimage(self.map_img_np)
		# self.path_img.show()

	def plan_path(self):
		self.path.clear_path() 								# clears the previous path each time a new goat point is given
		infy = 9999 										# sets the varible infy that represents infinity to 9999
		Array_1 = np.full_like(self.costmap.costmap, infy) 	# initializes Array 1 to be 9999 or infinity that is the same size of the cost array, This Array stores the value of the element
		num_of_rows = Array_1.shape[0]						# calculates the number of rows in Array_1
		num_of_columns = Array_1.shape[1]					# calculates the number of columns in Array_1
		max_el_row = num_of_rows - 1						# calculates the maximum row # in the Array (ie 0 = 1st row)
		max_el_col = num_of_columns - 1						# calculates the maximum column # in the Array (ie 0 = 1st column)
		priority_queue = [] 								# initializes Priority Queue
		start = (self.start_state_map.map_i, self.start_state_map.map_j) # sets the start point
		end = (self.goal_state_map.map_i, self.goal_state_map.map_j)     # sets the end point (row # of Array_1, column # of Array_1)
		heapq.heappush(priority_queue, (0, start[0], start[1], 0))       # Push the first element or starting point into the priority queue: (priority, x, y, g)
		points = [] 													 # initializes array storing the point that are on the way to goal position
		mu = .5															 # sets the value of mu in the formula used to calculate the element value (note: this will need to be tuned)
		row_from = np.full_like(self.costmap.costmap, infy) 			 # array that stores row number of the previous element for the current element
		column_from = np.full_like(self.costmap.costmap, infy) 			 # array that stores column number of the previous element for the current element
		column_from[start[1]][start[0]] = start[0]						 # sets the column # of the starting point in the array that is used for backtracing
		row_from[start[1]][start[0]] = start[1]							 # sets the row # of the starting point in the array that is used for backtracing
		positions_explored = 0											 # initializes the positions explored to 0 since no positions have been explored yet

		while priority_queue:											 # While loop that continue until the priority queue is empty or the break is reached
			f, x, y, g = heapq.heappop(priority_queue)                   # Pop the element in the priority queue with the smallest value (Note: x = column # & y = row #)
			positions_explored = positions_explored + 1					 # adds 1 to the position explored each time a element is poped
			popped_row = y 												 # variable that stores the row # of the popped element from the priority queue
			popped_column = x 											 # variable that stores the column # of the popped element from the priority queue

			if (y, x) == end:                                                                # if the popped element is the same as the goal element
				print(positions_explored)												     # print how many elements had to be explored to get to the goat element
				back_track_row =y															 # let the y be the final back tracking position for row
				back_track_column =x														 # let the x be the final back tracking position for column
				while (back_track_row, back_track_column) != (start[1], start[0]):           # While loop that keeps going until the starting point is reached
					points.append((int(back_track_row), int(back_track_column))) 			 # Add the back tracked row and column to array storing back track path
					back_track_row_2 = int(back_track_row)									 # temp store the integer row value
					back_track_column_2 = int(back_track_column)						     # temp store the integer column value
					back_track_row = row_from[back_track_row_2, back_track_column_2]		 # set the new back track row # to the parent row number
					back_track_column= column_from[back_track_row_2, back_track_column_2]    # set the new back track column # to the parent column number
					
				break 																		 # breaks the while loop when path has been created

			
			if 0 < x < max_el_col and 0 < y < max_el_row:     # if the element column # and row # are with in the limits of the array
				for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]: # Explore neighbors (right, left, down, up) 
					nx, ny = x + dx, y +dy # coordinate (ny, nx) is the coordinate of the neigbor being explored in Array_1
									   # Note: Array_1's elements are called out as (row#, column#) where the top left corner is (0, 0)
									   # Note: when dx is being called out, it is refering the change of the x-axis (horizontal) which means a change in the column # in Array_1
									   # Note: If dx is (+), Array_1's column # is increasing (going rightward elementwise) and opposite if negative
									   # Note: when dy is being called out, it is refering the change of the y-axis (vertical) which means a change in the row # in Array_1
									   # Note: If dy is (+), Array_1's row # is decreasing (going going upward elementwise) and opposite if negative
					if row_from[ny][nx] == infy:
						row_from[ny][nx] = popped_row

					if column_from[ny][nx] == infy:
						column_from[ny][nx] = popped_column

					if Array_1[ny][nx] == infy:                                  # if the element in the in Array_1 hasnt been popped perform below
						g_new = self.costmap.costmap[ny][nx]                     # extract the costmap value calculated from cost_map.py @ the neigbor coordinate (ny, nx) known as "g" or "c" of the Heuristics formula
					    #h_new = (abs(end[0] - ny) + abs(end[1] - nx))           # calculate the H of the Heuristics formula using the Manhattan method
						h_new = math.sqrt((end[0] - ny)**2 + (end[1] - nx)**2)   # calculate the H of the Heuristics formula using the Euclidean method
						f_new = g_new + mu * h_new                               # calculate the value of the Heuristics formula with mu included in the formula
						Array_1[ny][nx] = f_new                                  # put the Heuristic formula value into Array_1
						heapq.heappush(priority_queue, (f_new, nx, ny, g_new))   # push the element into the priority queue

		for p in points: 														 # plots all points to create the path
			self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0)) 			 # adds plot to figure that is outputted
	
		self.path.save_path(file_name="Log\ddd.csv") # should be the csv file

# bresenham algorithm for line generation on grid map
# from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
def bresenham(x1, y1, x2, y2):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions

    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
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
