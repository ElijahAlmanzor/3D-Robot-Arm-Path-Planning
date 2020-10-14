#!/usr/bin/env python

import rospy
import intera_interface
import numpy as np
import math
import os
from subprocess import Popen
import time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import pickle


#Importing relevant ROS msg and srv files
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
from geometry_msgs.msg import(
     Point,
     Quaternion,
     Pose,
     PoseStamped
)


class SawyerAntColony :
     def __init__(self, evaporationFactor, pheromoneConstant, no_iterations, no_ants, no_movements):   
          '''Variables regarding the algorithm'''
          self.POSSIBLEMOVEMENTS = 12
          self.INVALIDPATHCOST = 10000
          self.MINORPATHCOST = 30
          self.MAJORPATHCOST = 1000
          #Capped variables signify values that should not change
          self.evaporationFactor = evaporationFactor
          self.pheromoneConstant = pheromoneConstant
          self.no_iterations = no_iterations
          self.no_ants = no_ants
          self.no_movements = no_movements
          self.pheromoneMatrix = np.ones((self.no_movements,self.POSSIBLEMOVEMENTS), dtype = float)
          #Creates an array of float 1's with row of number of moves and columns of the possible movements
          self.cost = [0.0 for i in range(self.no_ants)]
          #Creates an array to hold the cost value for each ant
          self.movementList = [[] for i in range(self.no_ants)]
          #Array to hold the movements for each ant
          self.bestMovement = []
          self.bestCost = self.MAJORPATHCOST #Just initial value to improve
          self.final_position_reached = False
          self.cost_flag = 0
          self.initial_cost = 0.0
          self.continueIt = 0
          #Flags for whether collision occur - minor or major
          '''Variables regarding the Sawyer robot and Gazebo'''
          self.limb = intera_interface.Limb('right')
          self.gripper = intera_interface.Gripper()
          self.pose = Pose() #Stores the current pose
          self.start_pos = [0.804095035604646, 0.40375037074549019, 0.43086834806251317]
          self.start_ori = [0.5119984445075462, 0.5014614863458349, 0.4072369351957381, 0.566173161803049]
          self.fin_pos = [0.7917929378700504, -0.11948517908403811, 0.8281656432612197]
          self.fin_ori_q = [0.21483347686974602, 0.09789468815244694, 0.48805427625647435, 0.8402774724311969]
          self.final_pose = Pose()
          self.reset = rospy.ServiceProxy('/gazebo/reset_world', Empty)
          self.subscriber_obs_pos = None
          '''Variables regarding the path minimisation'''
          self.coordinates_list = [] #stores the coordinates of each path
          self.reduced_coordinates = []
          self.temp_move_list = []
          self.store_path_flag = False
          self.temp_move = []
          self.temp_cost = 0.0
     
     def select_next_move(self,movement):
          '''Selects the next move based on their pheromone probabilities'''
          sumPheromone = sum(movement)
          probability_list = []
          movement_list = range(12)
          for pheromone in movement:
               probability_list.append((pheromone**1.3)/sumPheromone)
          return np.random.choice(movement_list, 1, probability_list)[0]
          
     

     
     def initialise_pose(self):
          self.limb.move_to_neutral()
          start_pose = Pose()
          start_pose.position.x = self.start_pos[0]
          start_pose.position.y = self.start_pos[1]
          start_pose.position.z = self.start_pos[2]
          start_pose.orientation.x = self.start_ori[0]
          start_pose.orientation.y = self.start_ori[1]
          start_pose.orientation.z = self.start_ori[2]
          start_pose.orientation.w = self.start_ori[3]
          self.moveTo(start_pose)
          self.reset() #reset the world and obstacles
          
          #pose holds the current position of the gripper tip
          current_pose = self.limb.endpoint_pose()
          self.pose.position.x = current_pose['position'].x
          self.pose.position.y = current_pose['position'].y
          self.pose.position.z = current_pose['position'].z 
          self.pose.orientation.x = current_pose['orientation'].x
          self.pose.orientation.y = current_pose['orientation'].y
          self.pose.orientation.z = current_pose['orientation'].z
          self.pose.orientation.w = current_pose['orientation'].w
          self.reset()
          #Final position to move to
          self.final_pose.position.x = self.fin_pos[0]
          self.final_pose.position.y = self.fin_pos[1]
          self.final_pose.position.z = self.fin_pos[2]
          self.final_pose.orientation.x = self.fin_ori_q[0]
          self.final_pose.orientation.y = self.fin_ori_q[1]
          self.final_pose.orientation.z = self.fin_ori_q[2]
          self.final_pose.orientation.w = self.fin_ori_q[3]          

          self.coordinates_list = [] 
          self.reduced_coordinates = []
          #ensures empty at the start
          
          
     def path_cost(self):
          change_in_x = abs(self.fin_pos[0] - self.pose.position.x)
          change_in_y = abs(self.fin_pos[1] - self.pose.position.y)
          change_in_z = abs(self.fin_pos[2] - self.pose.position.z)
          current_ori = list(euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w]))
          fin_ori = list(euler_from_quaternion([self.final_pose.orientation.x,self.final_pose.orientation.y,self.final_pose.orientation.z,self.final_pose.orientation.w]))
          change_in_euler_x = abs(fin_ori[0] - current_ori[0])
          change_in_euler_y = abs(fin_ori[1] - current_ori[1])
          change_in_euler_z = abs(fin_ori[2] - current_ori[2])
          cur_path_cost = math.sqrt((change_in_x**2) + (change_in_y**2) + (change_in_z**2) + (change_in_euler_x**2) + (change_in_euler_y**2) + (change_in_euler_z**2))
          return cur_path_cost
     
     
     def moveTo(self, next_pose):
          limb_joints = self.limb.ik_request(next_pose, "right_gripper_tip")
          #Right gripper tip takes the gripper tip into account- no need for the z offset
          if limb_joints != False:
               self.limb.move_to_joint_positions(limb_joints)
               return True
          else: 
               return False
     
     
     def moveSawyer(self,index):
          if index == 0:
               self.pose.position.x += 0.1
          elif index == 1:
               self.pose.position.x -= 0.1
          elif index == 2:
               self.pose.position.y += 0.1
          elif index == 3:
               self.pose.position.y -= 0.1
          elif index == 4:
               self.pose.position.z += 0.1
          elif index == 5:
               self.pose.position.z -= 0.1
          elif index == 6:
               temp_euler = list(euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w]))
               temp_euler[0] += 0.1#15 degrees movement
               temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
               self.pose.orientation.x = temp_ori[0]
               self.pose.orientation.y = temp_ori[1]
               self.pose.orientation.z = temp_ori[2]
               self.pose.orientation.w = temp_ori[3]
          elif index == 7:
               temp_euler = list(euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w]))
               temp_euler[0] -= 0.1 #10 degrees movement
               temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
               self.pose.orientation.x = temp_ori[0]
               self.pose.orientation.y = temp_ori[1]
               self.pose.orientation.z = temp_ori[2]
               self.pose.orientation.w = temp_ori[3]
          elif index == 8:
               temp_euler = list(euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w]))
               temp_euler[1] += 0.1
               temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
               self.pose.orientation.x = temp_ori[0]
               self.pose.orientation.y = temp_ori[1]
               self.pose.orientation.z = temp_ori[2]
               self.pose.orientation.w = temp_ori[3]
          elif index == 9:
               temp_euler = list(euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w]))
               temp_euler[1] -= 0.1
               temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
               self.pose.orientation.x = temp_ori[0]
               self.pose.orientation.y = temp_ori[1]
               self.pose.orientation.z = temp_ori[2]
               self.pose.orientation.w = temp_ori[3]
          elif index == 10:
               temp_euler = list(euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w]))
               temp_euler[2] += 0.1
               temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
               self.pose.orientation.x = temp_ori[0]
               self.pose.orientation.y = temp_ori[1]
               self.pose.orientation.z = temp_ori[2]
               self.pose.orientation.w = temp_ori[3]
          elif index == 11:
               temp_euler = list(euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w]))
               temp_euler[2] -= 0.1
               temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
               self.pose.orientation.x = temp_ori[0]
               self.pose.orientation.y = temp_ori[1]
               self.pose.orientation.z = temp_ori[2]
               self.pose.orientation.w = temp_ori[3]
          
          if self.moveTo(self.pose):
               return True
          else: 
               return False
     
     
     
     def callback(self,msg):
	#Change in position for the table
          change_in_x_table = abs(0.0 - msg.pose[2].position.x)
          change_in_y_table = abs(0.0 - msg.pose[2].position.y)
          change_in_z_table = abs(0.0 - msg.pose[2].position.z)
	#Change in position for the car window
          change_in_x_lobs = abs(0.0 - msg.pose[3].position.x)
          change_in_y_lobs = abs(0.0 - msg.pose[3].position.y)
          change_in_z_lobs = abs(0.0 - msg.pose[3].position.z)



	     
          if (change_in_x_table > 0.05 and change_in_x_table <= 0.1) or (change_in_y_table > 0.05 and change_in_y_table <= 0.1) or (change_in_z_table > 0.05 and change_in_z_table <= 0.1):
               self.cost_flag = 1	
               
          if (change_in_x_lobs > 0.05 and change_in_x_lobs <= 0.1) or (change_in_y_lobs > 0.05 and change_in_y_lobs <= 0.1) or (change_in_z_lobs > 0.05 and change_in_z_lobs <= 0.1):
               self.cost_flag = 1	

		
          if (change_in_x_table > 0.1) or (change_in_y_table > 0.1) or (change_in_z_table > 0.1):
               self.cost_flag = 2
             
          if (change_in_x_lobs > 0.1) or (change_in_y_lobs > 0.1) or (change_in_z_lobs > 0.1):
               self.cost_flag = 2
          #relatively high tolerances for moving because of the car door wobble
          
     def Initial_costFunction(self, cur_ant, cur_iteration):
          
          cur_path_length = len(self.movementList[cur_ant])          
          cur_path_cost = self.path_cost() #Costs the current path moving from last move to finish position
          print("> Vector length to desired point before reduction")
          print("\t%f"%cur_path_cost)
          self.cost[cur_ant] += ((cur_path_cost) + cur_path_length)      
              
          if self.moveTo(self.final_pose) == False:
               print("> Invalid path moving from last position to desired position")
               #Once the desired number of movement has been done, move it to the desired position
               self.cost[cur_ant] += self.INVALIDPATHCOST
               return False
          if self.cost_flag == 1:
               print("> Minor Collision Occured")
               self.cost[cur_ant] += self.MINORPATHCOST
               self.temp_move = self.movementList[cur_ant]
               self.temp_cost = self.cost[cur_ant]
               print("****************************")
               return True
          elif self.cost_flag == 2:
               print("> Major Collision Occured")
               self.cost[cur_ant] += self.MAJORPATHCOST
               self.cost_flag = 0
               print("****************************")
               return False
          else:
               print("> No collision has occured")
               self.temp_move = self.movementList[cur_ant]
               self.temp_cost = self.cost[cur_ant]
               print("****************************")
               return True
          #returns false if any of the moves are invalid

     def pheromoneEvaporation(self):     
          for cur_ant in range(self.no_ants):
               for index, cur_move in enumerate(self.movementList[cur_ant]):
                    self.pheromoneMatrix[index, cur_move] += (self.pheromoneConstant/self.cost[cur_ant])
                    #Update the pheromone matrix for the paths that have been taken
          self.pheromoneMatrix = self.pheromoneMatrix * (1.0 - self.evaporationFactor)
          #Update the overall matrix pheromone for the next bath of ants
          self.cost = [0.0 for i in range(self.no_ants)]
          self.movementList = [[] for i in range(self.no_ants)]
          #Reinitiliase the cost and movement holder for each ant
          if self.bestCost < self.MAJORPATHCOST:
               self.no_movements = len(self.bestMovement)
               print("New move limit %d" %(self.no_movements))
               print("------------------------------------------------------------------------------")
               path_write = open("cost.txt", "a")
               path_write.write(str(self.bestCost))
               path_write.write("\n")
               path_write.close()
               
               
     def store_coordinates(self):
          coordinate_endpoint = self.limb.endpoint_pose()
          coordinate_ph = Pose()
          coordinate_ph.position.x = coordinate_endpoint['position'].x
          coordinate_ph.position.y = coordinate_endpoint['position'].y
          coordinate_ph.position.z = coordinate_endpoint['position'].z
          coordinate_ph.orientation.x = coordinate_endpoint['orientation'].x
          coordinate_ph.orientation.y = coordinate_endpoint['orientation'].y
          coordinate_ph.orientation.z = coordinate_endpoint['orientation'].z
          coordinate_ph.orientation.w = coordinate_endpoint['orientation'].w
          self.coordinates_list.append(coordinate_ph)     
     
     def comparePose(self,cur_ind):
          red_coord_orientation = list(euler_from_quaternion([self.reduced_coordinates[-1].orientation.x,self.reduced_coordinates[-1].orientation.y,self.reduced_coordinates[-1].orientation.z,self.reduced_coordinates[-1].orientation.w]))
          coor_orientation = list(euler_from_quaternion([self.coordinates_list[cur_ind].orientation.x,self.coordinates_list[cur_ind].orientation.y,self.coordinates_list[cur_ind].orientation.z,self.coordinates_list[cur_ind].orientation.w]))
          change_in_x = abs(abs(self.reduced_coordinates[-1].position.x) - abs(self.coordinates_list[cur_ind].position.x))
          change_in_y = abs(abs(self.reduced_coordinates[-1].position.y) - abs(self.coordinates_list[cur_ind].position.y))
          change_in_z = abs(abs(self.reduced_coordinates[-1].position.z) - abs(self.coordinates_list[cur_ind].position.z))
          change_in_x_euler = abs(abs(red_coord_orientation[0]) - abs(coor_orientation[0]))
          change_in_y_euler = abs(abs(red_coord_orientation[1]) - abs(coor_orientation[1]))
          change_in_z_euler = abs(abs(red_coord_orientation[2]) - abs(coor_orientation[2]))
          
          change = math.sqrt(change_in_x**2 + change_in_y**2 + change_in_z**2 + change_in_x_euler**2 + change_in_y_euler**2 + change_in_z_euler**2)
          if change < 0.11:
		#If this subsequent move is postionally and orientationally less less than one move then consider it not moved at all
               return True          
     
     def reducepath(self,cur_ant):
          self.temp_move_list = []
          index = 0
          prev_stored_index = 0
          one_move_flag = False #flag to determine if a move relative to the current comparison is only one move away
          self.reduced_coordinates.append(self.coordinates_list[0])
          #Adds the very first value to the reduced coordinates
          while index < (len(self.coordinates_list)):
               for cur_coordinate_index, coordinate in enumerate(self.coordinates_list[(index+2):], index+2):
                    if self.comparePose(cur_coordinate_index):
                         #print("Current index %d is one step away from compare index %d" % (cur_coordinate_index, index))
                         index = cur_coordinate_index
                         prev_stored_index = cur_coordinate_index
                         self.reduced_coordinates.append(self.coordinates_list[cur_coordinate_index])
                         one_move_flag = True
                         break
               if one_move_flag == False:
                    if not index == prev_stored_index:
                         self.reduced_coordinates.append(self.coordinates_list[index])
                    index += 1
               one_move_flag = False
          #After all this, reduced_coordinates should have all the new coordinates list
          
          coordinate_index = 0
          self.temp_move_list.append(self.movementList[cur_ant][0])
          for coordinate_index, coordinate in enumerate(self.reduced_coordinates[(coordinate_index+1):], coordinate_index+1):
               cix = coordinate.position.x - self.reduced_coordinates[coordinate_index-1].position.x
               ciy = coordinate.position.y - self.reduced_coordinates[coordinate_index-1].position.y
               ciz = coordinate.position.z - self.reduced_coordinates[coordinate_index-1].position.z
               cur_euler = list(euler_from_quaternion([coordinate.orientation.x,coordinate.orientation.y,coordinate.orientation.z,coordinate.orientation.w]))
               prev_euler = list(euler_from_quaternion([self.reduced_coordinates[coordinate_index-1].orientation.x,self.reduced_coordinates[coordinate_index-1].orientation.y,self.reduced_coordinates[coordinate_index-1].orientation.z,self.reduced_coordinates[coordinate_index-1].orientation.w]))               
               cixe = cur_euler[0] - prev_euler[0]
               ciye = cur_euler[1] - prev_euler[1]
               cize = cur_euler[2] - prev_euler[2]
    

               if cix < 1.1 and cix > 0.09:
                    self.temp_move_list.append(0)
                    
               elif cix > -1.1 and cix < -0.09:
                    self.temp_move_list.append(1)
                    
               elif ciy < 1.1 and ciy > 0.09:
                    self.temp_move_list.append(2)
                    
               elif ciy > -1.1 and ciy < -0.09:
                    self.temp_move_list.append(3)	
                    
               elif ciz < 1.1 and ciz > 0.09:
                    self.temp_move_list.append(4)
                    
               elif ciz > -1.1 and ciz < -0.09:
                    self.temp_move_list.append(5)		
                   
               elif cixe < 1.025 and cixe > 0.0975:
                    self.temp_move_list.append(6)
                    
               elif cixe > -1.025 and cixe < -0.0975:
                    self.temp_move_list.append(7)	
                    
               elif ciye < 1.025 and ciye > 0.0975:
                    self.temp_move_list.append(8)
                    
               elif ciye > -1.025 and ciye < -0.0975:
                    self.temp_move_list.append(9)
                    
               elif cize < 1.025 and cize > 0.0975:
                    self.temp_move_list.append(10)
                    
               elif cize > -1.025 and cize < -0.0975:
                    self.temp_move_list.append(11)	
                    
          
          
          print("> Move list before reduction")
          print(self.movementList[cur_ant])
          print("> Move list after reduction")
          print(self.temp_move_list)
          
          if len(self.temp_move_list) < len(self.movementList[cur_ant]):
               self.movementList[cur_ant] = self.temp_move_list
               self.temp_move_list = []
               self.coordinates_list = []
               self.reduced_coordinates = []
               print("****************************")
               return True
          else:
               print("> Path length was not reduced, keeping pre-reduction solution found")
               self.temp_move_list = []
               self.coordinates_list = []
               self.reduced_coordinates = []
               #reset everything back to empty lists
               print("****************************")
               return False

     def Reduced_costFunction(self, cur_ant, cur_iteration):
          #now basically everything runs twice
          self.initialise_pose()
          self.cost_flag = 0
          for move in self.movementList[cur_ant]:
               if not self.moveSawyer(move):
                    print("> Became an invalid path after reduction - reverting back to pre-reduction cost") #write a code that reverts it back
                    self.cost[cur_ant] = self.temp_cost
                    self.movementList[cur_ant] = self.temp_move
                    return False
                    
          self.cost[cur_ant] = 0.0      #Reset the cost function if it has reached this bit     
          cur_path_length = len(self.movementList[cur_ant])          
          cur_path_cost = self.path_cost() #Costs the current path moving from last move to finish position
          print("> Vector distance to desired point after reduction")
          print("\t%f"%cur_path_cost)
          self.cost[cur_ant] += ((cur_path_cost) + cur_path_length)      
              
          if self.moveTo(self.final_pose) == False:
               print("> Invalid path moving from last position to desired position")
               #Once the desired number of movement has been done, move it to the desired position
               self.cost[cur_ant] += self.INVALIDPATHCOST
          #To stop the callback changing the cost flag
          
          
          if self.cost_flag == 1:
               print("> Minor Collision Occured After Path Reduction")
               self.cost[cur_ant] += self.MINORPATHCOST
          elif self.cost_flag == 2:
               print("> Major Collision Occured After Path Reduction")
               self.cost[cur_ant] = self.MAJORPATHCOST
          else:
               print("> No collision has occured After Path Reduction")
          print("*********************************************")
		
          print("> Pre-reduction cost value")
          print("\t%f"%self.temp_cost)
          print("> Post-reduction cost value")
          print("\t%f"%self.cost[cur_ant])
          if self.temp_cost < self.cost[cur_ant]:
               print("> Since reduced path has higher cost, reverting to pre-reduction path cost")
               self.cost[cur_ant] = self.temp_cost
               self.movementList[cur_ant] = self.temp_move
               print("\t%f"%(self.cost[cur_ant]))
          print("****************************")
          
          
     def display_information(self,cur_ant,cur_iteration):                    
          if self.cost[cur_ant] < self.bestCost:
               self.bestCost = self.cost[cur_ant]
               self.bestMovement = self.movementList[cur_ant]
                         

               #path_write = open("path.txt", "a")
               #path_write.write("[")
               #for i, element in enumerate(self.bestMovement):
               #     path_write.write("%s" % element)
               #     if i < (len(self.bestMovement) - 1):
                #         path_write.write(",")
               #path_write.write("]")
               #path_write.write("\n")
               #path_write.close()
    
          if self.bestCost < self.MAJORPATHCOST:               
               print("> Current Best Cost")
               print(self.bestCost)
               print("> Current Best Path")
               print(self.bestMovement)
               
          else:
               print("> No solutions found yet")
               
          print("*************************")     

          
          print("> Current Overall Path Cost")
          print("\t%f"%self.cost[cur_ant])   
                    
          print("> Current Iteration")
          print("\t%d"%cur_iteration)
          print("> Current Ant")
          print("\t%d"%cur_ant)
          print("------------------------------------------------------------------------------")
          self.subscriber_obs_pos.unregister()
          #Stops recording any changes to obstacles
          self.cost_flag = 0
          #Reset the cost flag for the next ant
          self.store_path_flag = False
          #Turns of the reduction flag
          self.temp_move = []
          self.temp_cost = 0.0

     def writeP(self,cit):
          save = np.savetxt('GACO.txt', self.pheromoneMatrix)
          #print current iteration
          it_file = open("cur_iteration.txt",'w')
          it_file.write(str(cit))
          it_file.close()
          #print current best cost
          cost_file = open("bestCost.txt",'w')
          cost_file.write(str(self.bestCost))
          cost_file.close()
          #print current best move length
          length_file = open("moveLength.txt",'w')
          length_file.write(str(self.no_movements))
          length_file.close()
          #print current best path
          with open("Bestmove.txt", 'w') as output:
               pickle.dump(self.bestMovement,output)
    
     def continueFromCrash(self):
          self.pheromoneMatrix = np.loadtxt('GACO.txt', delimiter=" ")
          
          it_file = open("cur_iteration.txt", 'r')
          self.continueIt = int(it_file.read())
          it_file.close()
          
          cost_file = open("bestCost.txt",'r')
          self.bestCost = float(cost_file.read())
          cost_file.close()

          length_file = open("moveLength.txt",'r')
          mvmt = length_file.read()
          self.no_movements = int(mvmt)
          length_file.close()

          
          with open("Bestmove.txt", 'r') as infile:
               self.bestMovement = pickle.load(infile)
     
     
     
     def reset_txt(self):
          self.pheromoneMatrix = np.ones((self.no_movements,self.POSSIBLEMOVEMENTS), dtype = float)          
          cit = 0
          self.bestCost = self.MAJORPATHCOST
          self.bestMovement = []
          
          save = np.savetxt('GACO.txt', self.pheromoneMatrix)
          #print current iteration
          it_file = open("cur_iteration.txt",'w')
          it_file.write(str(cit))
          it_file.close()
          #print current best cost
          cost_file = open("bestCost.txt",'w')
          cost_file.write(str(self.bestCost))
          cost_file.close()
          #print current best path
          with open("Bestmove.txt", 'w') as output:
               pickle.dump(self.bestMovement,output)
          
          length_file = open("moveLength.txt",'w')
          length_file.write(str(self.no_movements))
          length_file.close()
          
          
          
          
     def main(self):
          self.initialise_pose()
          
          for cur_iteration in range(self.continueIt, self.no_iterations):
               self.writeP(cur_iteration)
               for cur_ant in range(self.no_ants):
                    self.initialise_pose()
                    #time.sleep(1)
                    #resets the Gazebo simulation
                    self.subscriber_obs_pos = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
                    #Detects movement of the obstacles if collision has occured 
                    for cur_move in range(self.no_movements):
                         next_move = self.select_next_move(self.pheromoneMatrix[cur_move,:])
                         #time.sleep(1)
                         #Passes the current row of possible moves into the random move generator function
                         self.movementList[cur_ant].append(next_move)
                         #time.sleep(1)
                         #Appends the chosen move to the movementList
                         if (self.moveSawyer(next_move) == False):
                              #time.sleep(2)
                         #Function returns false if the next move is invalid - not physically possible
                              print("Invalid Path")
                              self.cost[cur_ant] += self.INVALIDPATHCOST
                              self.store_path_flag = False
                              break
                              #Skips the current ant if it involves an invalid path and add a very high cost value
                         else:
                              self.store_coordinates()
                              self.store_path_flag = True
                              #Don't need to improve the solution if the move is invalid!
                         #rospy.sleep(3)
                    if self.store_path_flag and self.Initial_costFunction(cur_ant,cur_iteration):
                         #time.sleep(1)
                         #rospy.sleep(3)
                         if self.reducepath(cur_ant):
                         	self.Reduced_costFunction(cur_ant,cur_iteration)
                    self.display_information(cur_ant,cur_iteration)
                    #time.sleep(1)
                    #rospy.sleep(3)
                    #Needs to cost the function even if it was improved or not

               self.pheromoneEvaporation()     
               
			#Updates the pheromones of the solution
			#write out the best solution cost so can plot over time

'''---------------------------------------------------------------------------------------------------'''


if __name__ == '__main__':
     try:
          proc = Popen(['killall gzserver && killall gzclient'], shell=True, stdin = None, stdout = None, stderr = None, close_fds = True)
          proc = Popen(['roslaunch sawyer_gazebo sawyer_world.launch'], shell=True, stdin = None, stdout = None, stderr = None, close_fds = True)
          for sec in xrange(5, 0, -1):
                    print("%d seconds left before starting the main program"%sec)
                    time.sleep(1)
          rospy.init_node('GACO')
          aco = SawyerAntColony(0.1, 1.0, 25, 75, 15)
          #aco.reset_txt()
          aco.continueFromCrash()
          aco.main()
     except rospy.ROSInterruptException:
          print("Program has been interrupted")
