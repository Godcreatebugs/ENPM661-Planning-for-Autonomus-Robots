# -*- coding: utf-8 -*-
"""


@author: Piyush Bhuva
"""

#importing packages

import time
import collections

radius=input("Put  radius: ")
radius=int(radius)
clearance=input("Put clearance: ")
clearance=int(clearance)
resolution=input("Put  resolution: ")
resolution=float(resolution)
start=input("starting node separated by comma: ")
start=start.split(',')
start = list(map(int, start))
start[0]=int(start[0]/resolution)
start[1]=int(start[1]/resolution)
"""Very important thing to declare:
    I have used radius to define a point and a rigid robot (Tried and failed for a differnet method)
    so for an instance for point robot put radius=0
                       for rigid robot(Circular Robot) put radius =integer as a radius"""
       
"""
The direction to goal will be defined via distance in each axis and that is why 
I have seperated out as separating out x and y coordinate for each point
resolution is for how far you can go meaning setting resolution at 2 measns that you can go half to goal
"""  


                
while True:
    if start[0]<0 or start[0]>int(249/resolution) or start[1]<0 or start[1]>int(149/resolution):
        print("Pinput invalid")
        break
    goal=input("Goal node separated by , : ")
    goal=goal.split(',')
    goal = list(map(int, goal))
    goal[0]=int(goal[0]/resolution)
    goal[1]=int(goal[1]/resolution)
    while True:
        if goal[0]<0 or goal[0]>int(249/resolution) or goal[1]<0 or goal[1]>int(149/resolution):
            print("Try again")
            break

    

        start_time = time.time()
        import Dji_map
        import math
        import time

        graph=Dji_map.Obstacle_map(radius, clearance, resolution)
#print(graph)
        if tuple(goal) in graph.keys()==False:
                print("Goal is in obstacle space")
           
        def dijkstra(graph,start,goal):
            shortest_distance = {}
            predecessor = {}
            exploredNodes= collections.OrderedDict()
            unseenNodes = graph
            infinity = math.inf
            path = []
            for node in unseenNodes:
                shortest_distance[node] = infinity
            shortest_distance[start] = 0
            while unseenNodes:
                minNode = None
                for node in unseenNodes:
                    if minNode is None:
                        minNode = node
                    elif shortest_distance[node] < shortest_distance[minNode]:
                        minNode = node
         
                for childNode, weight in graph[minNode].items():
                    if weight + shortest_distance[minNode] < shortest_distance[childNode]:
                        shortest_distance[childNode] = weight + shortest_distance[minNode]
                        predecessor[childNode] = minNode
                exploredNodes.update(unseenNodes.pop(minNode))
         
            currentNode = goal
            while currentNode != start:
                try:
                    path.insert(0,currentNode)
                    currentNode = predecessor[currentNode]
                except KeyError:
                    print('Path not reachable')
                    break
            path.insert(0,start)
            if shortest_distance[goal] != infinity:
                print('Shortest distance is ' + str(shortest_distance[goal]))
                #print('And the path is ' + str(path))
                open('path_dij.txt', 'w').write('\n'.join('%s\t%s' % x for x in path))
                open('node_dij.txt', 'w').write('\n'.join('%s\t%s' % x for x in exploredNodes))
         
        dijkstra(graph, tuple(start), tuple(goal))
        end_time = time.time()
        print("Total time: "+str(end_time-start_time))
        break
    break