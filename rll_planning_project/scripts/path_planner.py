#! /usr/bin/env python

import rospy
import actionlib
from rll_planning_project.srv import *
from rll_planning_project.msg import *
from geometry_msgs.msg import Pose2D
import math

class Cell:
    # Class to hold info for each cell in the path
    def __init__(self, x_grid=None, y_grid=None, x_map=None, y_map=None, theta = 0.0):
       # x and y coordinates of cell in the grid and map coordinates
       self.x_grid = x_grid
       self.y_grid = y_grid
       self.y_map = y_map
       self.x_map = x_map

       # Gripper orientation on robot to get to this cell
       self.theta = theta

       self.previous_cell = None


       #Cost of getting to this cell to calculate path planning
       self.g = 0
       self.h = 0
       self.f = 0

# Convert grid coordinates to map coordinates to send a pose to the robot
def calculate_x_map_coordinate(x):
    x_map_coordinate = float(x)
         
    x_map_coordinate = x_map_coordinate - 6
    x_map_coordinate = x_map_coordinate / 10


    return x_map_coordinate

def calculate_y_map_coordinate(y):  
    y_map_coordinate = float(y)    
    y_map_coordinate = y_map_coordinate - 8
    y_map_coordinate = y_map_coordinate / 10

    return y_map_coordinate   
    

def calculate_x_grid_coordinate(x):
#Convert map coordinates to a grid position in order to implement A* path planning

    x = x * 10
    int_x = int(x)

    int_x = int_x + 6

    return int_x


def calculate_y_grid_coordinate(y):
    y = y * 10
    int_y = int(y)
     
    int_y = int_y + 8

    return int_y

def plan_to_goal(req):
    # Plan a path from start to goal
    pose_start = Pose2D()
    pose_goal = Pose2D()
    pose_check_start = Pose2D()
    pose_check_goal = Pose2D()
    pose_move = Pose2D()

    out_of_range = False
    blocked_path = False
 

    #Declare lists
    open_list = []
    closed_list = []
    path = []
    neighbor_cells = []
    find_neighbor_positions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
     

    pose_start = req.start
    pose_goal = req.goal

    move_srv = rospy.ServiceProxy('move', Move)
    check_srv = rospy.ServiceProxy('check_path', CheckPath, persistent=True)


    # Input: map dimensions, start pose, and goal pose
    # retrieving input values  
    map_width = rospy.get_param('~map_width')
    map_length = rospy.get_param('~map_length')

   
    #Grid Map dimensions
    map_grid_x = 12
    map_grid_y = 16

    xStart, yStart, tStart = pose_start.x, pose_start.y, pose_start.theta
    xGoal, yGoal, tGoal = pose_goal.x, pose_goal.y, pose_goal.theta
    # printing input values
    rospy.loginfo("map dimensions: width=%1.2fm, length=%1.2fm", map_width, map_length)
    rospy.loginfo("start pose: x %f, y %f, theta %f", xStart, yStart, tStart)
    rospy.loginfo("goal pose: x %f, y %f, theta %f", xGoal, yGoal, tGoal)
    
    # Initialize Cell class for starting and goal cells
    start_cell_x_grid = calculate_x_grid_coordinate(xStart)
    start_cell_y_grid = calculate_y_grid_coordinate(yStart)
    start_cell = Cell(start_cell_x_grid, start_cell_y_grid, xStart, yStart, tStart)
    start_cell.g = start_cell.h = start_cell.f = 0   
    rospy.loginfo("start cell: grid x %f, grid y %f, map x %f, map y %f", start_cell.x_grid, start_cell.y_grid, start_cell.x_map, start_cell.y_map)
    
    goal_cell_x_grid = calculate_x_grid_coordinate(xGoal)
    goal_cell_y_grid = calculate_y_grid_coordinate(yGoal)
    goal_cell = Cell(goal_cell_x_grid, goal_cell_y_grid, xGoal, yGoal, tGoal)
    goal_cell.g = goal_cell.h = goal_cell.f = 0

    rospy.loginfo("goal cell: grid x %f, grid y %f, map x %f, map y %f", goal_cell.x_grid, goal_cell.y_grid, goal_cell.x_map, goal_cell.y_map)
    # Add starting cell to the open list
    open_list.append(start_cell)

    # Loop through searching neighboring cells until the goal cell is found
    while len(open_list) > 0:
       
       # Find cell in open list with the lowest f cost
       lowest_f_cell = open_list[0]
       lowest_index = 0
       for i, current_cell in enumerate(open_list):
           if current_cell.f < lowest_f_cell:
                lowest_f_cell = current_cell
                lowest_index = i

       rospy.loginfo("lowest f cell: grid x %f, grid y %f, map x %f, map y %f", lowest_f_cell.x_grid, lowest_f_cell.y_grid, lowest_f_cell.x_map, lowest_f_cell.y_map)
       #Take the cell with the lowest f cost off of the open list and add it to the closed list
       #Find map coordinates of this cell and put them into a Pose2Drospy.loginfo("lowest_f_cell: grid x %f, grid y %f, 
       open_list.pop(lowest_index)
       closed_list.append(lowest_f_cell)
       pose_lowest_f_cell = Pose2D()
       pose_lowest_f_cell.x, pose_lowest_f_cell.y, pose_lowest_f_cell.theta = lowest_f_cell.x_grid, lowest_f_cell.y_grid, lowest_f_cell.theta
       
       rospy.loginfo("pose_lowest_f_cell: x %f, grid y %f, theta %f", pose_lowest_f_cell.x, pose_lowest_f_cell.y, pose_lowest_f_cell.theta)
       

       # If goal is found, stop searching and generate path from tracing back each cell parent
       if (lowest_f_cell.x_grid == goal_cell.x_grid) and (lowest_f_cell.y_grid == goal_cell.y_grid):
            path.append(goal_cell.x_map, goal_cell.y_map, goal_cell.theta)
         
            while current is not None:
                path.append(current.previous_cell.map_x, previous_cell.map_y, previous_cell.theta)
                current = current.previous_cell
            path.reverse()
        
       #Find neighboring cells to expand 
       for position_coordinates in find_neighbor_positions:
           neighbor_x_grid = lowest_f_cell.x_grid + position_coordinates[0]
           neighbor_y_grid = lowest_f_cell.y_grid + position_coordinates[1]
           neighbor_x_map = calculate_x_map_coordinate(neighbor_x_grid)
           neighbor_y_map = calculate_y_map_coordinate(neighbor_y_grid)

           rospy.loginfo("neighbor pose: x grid %f, x map %f, y grid %f, y map %f", neighbor_x_grid, neighbor_x_map, neighbor_y_grid, neighbor_y_map)

           #Checking to see if position is in the range of the grid
           if position_coordinates[0] > (map_grid_x - 1) or position_coordinates[0] < 0 or position_coordinates[1] > (map_grid_y - 1) or position_coordinates[1] < 0:
                out_of_range = True
           else:
                out_of_range = False

           #Checking to see if cell is blocked
           pose_neighbor = Pose2D()
           pose_neighbor.x, pose_neighbor.y = neighbor_x_map, neighbor_y_map
           pose_neighbor.theta = 0.0
           rospy.loginfo("pose_neighbor: x %f, grid y %f, theta %f", pose_neighbor.x, pose_neighbor.y, pose_neighbor.theta)

           resp = check_srv(pose_lowest_f_cell, pose_neighbor)

           if resp.valid:
               rospy.loginfo("Valid pose")
               blocked_path = False

           else:
               rospy.loginfo("Invalid pose")
               blocked_path = True

           if (out_of_range == False) and (blocked_path == False):
               neighbor = Cell(neighbor_x_grid, neighbor_y_grid, neighbor_x_map, neighbor_y_map, 0.00) 
               neighbor.previous_cell = lowest_f_cell
               neighbor_cells.append(neighbor)

       # Loop through the valid neighboring cells in the grid to find the one with the lowest g value that will be used in the
       # next iteration of the while loop
       for item in neighbor_cells:
            in_closed_list = False
            in_open_list = False

            # Inner for loop to see if item is also on the closed list
            for closed_cell in closed_list:
                if item == closed_cell:
                   in_closed_list = True

            if (in_closed_list == False):
               item.g = lowest_f_cell + 1 
               item.h = abs(item.x_grid - goal_cell.x_grid) + abs(item.y_grid - goal_cell.y_grid)
               item.f = item.g + item.h

               # Inner for loop to see if cell is already in the open list
               for open_list_cell in open_list:
                   if (item.x_grid == open_list_cell.x_grid) and (item.y_grid == open_list_cell.y_grid):
                       in_open_list = True

                       # Save the path with the lowest g value towards the cell
                       if item.g < open_list_cell.g:
                          open_list_cell.h = item.h
                          open_list_cell.f = item.f
                          open_list_cell.previous_cell = item.previous_cell
                          open_list_cell.theta = item.theta
                          open_list_cell.g = item.g

               if (in_open_list == False):
                   open_list.append(item) 


     # Loop to move robot arm through the path
    if path != None:
        for cell in path:
            pose_move.x, pose_move.y, pose_move.theta = cell[0], cell[1], cell[2]
            resp = move_srv(pose_move)
        else:
            rospy.loginfo("A path was not found.")

        


class PathPlanner:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("plan_to_goal", PlanToGoalAction, self.execute, False)
        self.server.start()

    def execute(self, req):
        plan_to_goal(req)
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('path_planner')

    server = PathPlanner()

    rospy.spin()
