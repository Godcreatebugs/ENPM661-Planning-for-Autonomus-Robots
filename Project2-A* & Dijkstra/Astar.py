# -*- coding: utf-8 -*-
"""


@author: Piyush Bhuva
"""
#importing packages
from __future__ import print_function
import Map_Astar as gr  #map for a star and using it(via elements)
import collections

#defining necessary terms to run a*
radius=input("Radius here: ") #radius as an input
radius=float(radius)
clearance=input("Put clearance: ")#clearance is the clearance from obstacle
clearance=float(clearance)
resolution=input("Put resolution: ") #how many steps at a time it takes (also affects the speed)
resolution=float(resolution)
start=input("startpoint separated by comma: ")  #start point
start=start.split(',')
start = list(map(int, start))
goal=input("goal point seperated by comma: ") #goal node
goal=goal.split(',')  #all input points are split by spcae in between
goal = list(map(int, goal))   #map function uses (data type , make a list)
"""
Very important thing to declare:
    I have used radius to define a point and a rigid robot (Tried and failed for a differnet method)
    so for an instance for point robot put radius=0
                       for rigid robot(Circular Robot) put radius =integer as a radius
"""                       
   

"""
The direction to goal will be defined via distance in each axis and that is why 
I have seperated out as separating out x and y coordinate for each point
resolution is for how far you can reach meaning path will be divided by 2 if resolution is 2
"""                    
start[0]=int(start[0]/resolution) #x co-ordinate of start and likwise
start[1]=int(start[1]/resolution)
goal[0]=int(goal[0]/resolution)
goal[1]=int(goal[1]/resolution)

"""
This is to check wether your point lies on a map of(150,250) co-ordinate or not
and if not printing that goal and start points are not in the map.
"""
while True: #first thing to check
    if start[0]<0 or start[0]>int(249/resolution) or start[1]<0 or start[1]>int(149/resolution):
        print("Print out points that lies on a plane.")
        break
    while True:
        if goal[0]<0 or goal[0]>int(249/resolution) or goal[1]<0 or goal[1]>int(149/resolution):
            print("Choose goal point that lies on a plane.")
            break
#if goal is in our map
 
        elements=gr.elements(radius,clearance,resolution)
        if tuple(goal) in elements==False:
                print("Goal is in obstacle space")
        class AStarMap(object):
            
            def heuristic(self, start, goal):
        #we can move in diagonal way or in side 4 direction
                dx= abs(start[0] - goal[0])
                dy= abs(start[1] - goal[1])
                return dx + dy
            
            def get_vertex_neighbours(self, pos):
                n= []  #list to store explored nodes
        		#this for loop allows us to move one step in one direction
                if pos in elements:
                    for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
                        x2= pos[0] + dx
                        y2 = pos[1] + dy
                        if (x2,y2) in elements:
                            n.append((x2, y2))
                return n
        ##checking optimality
            def move_cost(self, a, b):
                if(abs(a[0]-b[0])==1 and abs(a[1]-b[1])==1):
                    return 2**0.5
                elif (abs(a[0]-b[0])==0 or abs(a[0]-b[0])==1) and (abs(a[1]-b[1])==0 or abs(a[1]-b[1])==1):
                    return 1
         
        def AStar(graph, start, goal):
         
        	G = {} #Actual movement cost to each position from the start position
        	F = {} #Estimated movement cost of start to goal going via this position
         
        	#Initialize starting values
        	G[start] = 0 
        	F[start] = graph.heuristic(start, goal)
         
        	closedVertices = collections.OrderedDict()
        	openVertices = set([start])
        	cameFrom = {}
         
        	while len(openVertices) > 0:
        		#Get the vertex in the open list with the lowest F score
        		current = None
        		currentFscore = None
        		for pos in openVertices:
        			if current is None or F[pos] < currentFscore:
        				currentFscore = F[pos]
        				current = pos
         
        		#Check if we have reached the goal
        		if current == goal:
        			#Retrace our route backward
        			path = [current]
        			while current in cameFrom:
        				current = cameFrom[current]
        				path.append(current)
        			path.reverse()
        			return path, F[goal], closedVertices #Done!
         
        		#Mark the current vertex as closed
        		openVertices.remove(current)
        		closedVertices.update({current:0})
         
        		#Update scores for vertices near the current position
        		for neighbour in graph.get_vertex_neighbours(current):
        			if neighbour in closedVertices: 
        				continue #We have already processed this node exhaustively
        			candidateG = G[current] + graph.move_cost(current, neighbour)
         
        			if neighbour not in openVertices:
        				openVertices.add(neighbour) #Discovered a new vertex
        			elif candidateG >= G[neighbour]:
        				continue #This G score is worse than previously found
         
        			#Adopt this G score
        			cameFrom[neighbour] = current
        			G[neighbour] = candidateG
        			H = graph.heuristic(neighbour, goal)
        			F[neighbour] = G[neighbour] + H
             #in case no solution or long time run   
        	raise RuntimeError("A* failed to find a solution")
        break
    break
############################# END OF DECLARATION########################
    ##############EXECUTION WITH THE BELOW FUNCTION ################
    
#This function allows us to create a path file as txt for astar and also node exploration#
if __name__=="__main__":
    graph = AStarMap()
    result, cost, explored_nodes = AStar(graph, tuple(start), tuple(goal))
    open('node_astar.txt', 'w').write('\n'.join('%s\t%s' % x for x in explored_nodes))  #for node exploration
    open('path_astar.txt', 'w').write('\n'.join('%s\t%s' % x for x in result))        #for path
    print ("route", result) #path
    print ("cost", cost) #cost for steps
      
    
    
    
    