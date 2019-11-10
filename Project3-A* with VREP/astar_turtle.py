# author - Abhiram Dapke
# Execute this command in the child script of V-rep.
# simRemoteApi.start(19999)
# then we can start the simulation and run this program.


##importing modules
from operator import itemgetter
import math
import matplotlib.pyplot as plt

#obstacle space
def obstacle_map(x, y):
    offset = (0.354 / 2) + 0.1
    checkobsspace = 0
    
    if (x < -5.55 + offset) or (x > 5.55 - offset) or (y < -5.05 + offset) or (y > 5.05 - offset):
         checkobsspace = 1

    # Circular objects
    elif ((x + 1.65) ** 2 + (y + 4.6) ** 2) < (0.405 + offset) ** 2:
         checkobsspace = 1

    elif ((x + 1.17) ** 2 + (y + 2.31) ** 2) < (0.405 + offset) ** 2:
         checkobsspace = 1

    elif ((x + 1.17) ** 2 + (y - 2.31) ** 2) < (0.405 + offset) ** 2:
         checkobsspace = 1

    elif ((x + 1.65) ** 2 + (y - 4.6) ** 2) < (0.405 + offset) ** 2:
         checkobsspace = 1

    # Circular Table
    elif (((x + 4.05) ** 2 + (y - 3.25) ** 2) < (0.7995 + offset) ** 2) or (
            ((x + 2.46) ** 2 + (y - 3.25) ** 2) < (0.7995 + offset) ** 2):
         checkobsspace = 1

    elif (-4.05 - offset < x) and (x < -2.45 + offset) and (2.45 - offset < y) and (y < 4.05 + offset):
         checkobsspace = 1

    # Long Rectangle reaching
    elif (1.3 - offset < x) and (x < 5.55 + offset) and (-5.05 - offset < y) and (y < -4.7 + offset):  # 1
         checkobsspace = 1

    elif (-0.81 - offset < x) and (x < 1.93 + offset) and (-4.7 - offset < y) and (y < -3.18 + offset):  # 2
         checkobsspace = 1

    elif (2.24 - offset < x) and (x < 3.41 + offset) and (-4.7 - offset < y) and (y < -4.12 + offset):  # 3
         checkobsspace = 1

    elif (3.72 - offset < x) and (x < 5.55 + offset) and (-4.7 - offset < y) and (y < -3.94 + offset):  # 4
         checkobsspace = 1

    # Other Rectangles
    elif (-1.17 - offset < x) and (x < -0.26 + offset) and (-1.9 - offset < y) and (
            y < -0.08 + offset):  # A
         checkobsspace = 1

    elif (-0.26 - offset < x) and (x < 1.57 + offset) and (-2.4 - offset < y) and (y < -1.64 + offset):  # B
         checkobsspace = 1

    elif (2.29 - offset < x) and (x < 3.8 + offset) and (-2.38 - offset < y) and (y < -1.21 + offset):  # C
         checkobsspace = 1

    elif (4.97 - offset < x) and (x < 5.55 + offset) and (-3.27 - offset < y) and (y < -2.1 + offset):  # D
         checkobsspace = 1

    elif (4.64 - offset < x) and (x < 5.55 + offset) and (-1.42 - offset < y) and (y < -0.57 + offset):  # E
         checkobsspace = 1

    elif (4.97 - offset < x) and (x < 5.55 + offset) and (-0.57 - offset < y) and (y < 0.6 + offset):  # F
         checkobsspace = 1

    elif (1.89 - offset < x) and (x < 5.55 + offset) and (1.16 - offset < y) and (y < 1.92 + offset):  # G
         checkobsspace = 1

    elif (2.77 - offset < x) and (x < 3.63 + offset) and (3.22 - offset < y) and (y < 5.05 + offset):  # H
         checkobsspace = 1

    elif (4.28 - offset < x) and (x < 4.71 + offset) and (4.14 - offset < y) and (y < 5.05 + offset):  # I
         checkobsspace = 1

    return  checkobsspace


def check_points(node, points):
    checkobsspace = 0
    for point in points:
        if (((node[0] - point[0]) ** 2 + (node[1] - point[1]) ** 2) - 0.1 ** 2 < 0):
            return True
    return False

def hCost(node, goalNode):
    return (((node[0] - goalNode[0]) ** 2 + (node[1] - goalNode[1]) ** 2) ** (1 / 2))

#it is  a euclidian distance only
def cost(node, parentNode):
    return (((node[0] - parentNode[0]) ** 2 + (node[1] - parentNode[1]) ** 2) ** (1 / 2))


def action(node, leftwheel, rightwheel, currtheta):
    wl_rad = 0.076 / 2
    dist_bw_wl = 0.23

    temporary_x = node[0]
    temporary_y = node[1]
    temporary_theta = currtheta
    for i in range(1, 101):
        temporary_x = temporary_x + (wl_rad / 2) * (rightwheel + leftwheel) * (math.cos(temporary_theta)) * (2 / 100)
        temporary_y = temporary_y + (wl_rad / 2) * (leftwheel + rightwheel) * (math.sin(temporary_theta)) * (2 / 100)
        temporary_theta = temporary_theta + (wl_rad / dist_bw_wl) * (rightwheel - leftwheel) * (2 / 100)
        if obstacle_map(round(temporary_x, 1), round(temporary_y, 1)) == True:
            
            return math.floor(temporary_x*100)/100, math.floor(temporary_y*100)/100, temporary_theta

    
    return math.floor(temporary_x*100)/100, math.floor(temporary_y*100)/100, temporary_theta


def costupdated(child_node, goal_node, node, theta, action, all_nodes, cost_dict, p_id, action_dict, theta_dict):
    # Heuristic cost calculation between current and goal point
    heucost = hCost(node, goal_node)  
    cost_to_come = cost(node, child_node)
    if (child_node[0], child_node[1]) == (node[0], node[1]):

        return all_nodes, cost_dict, p_id, action_dict, theta_dict
   
    if (node[0], node[1]) not in list(
            cost_dict.keys()):  
        
        all_nodes.append(
            node + [cost_dict[(child_node[0], child_node[1])] + cost_to_come + heucost])  
        cost_dict[(node[0], node[1])] = cost_dict[(
        child_node[0], child_node[1])] + cost_to_come  
        p_id[(node[0], node[1])] = (
        child_node[0], child_node[1])  
        action_dict[(node[0], node[1])] = action
        theta_dict[(node[0], node[1])] = theta

    # cost for the child node
    elif cost_dict[(node[0], node[1])] + heucost > cost_dict[(
    child_node[0], child_node[1])] + cost_to_come + heucost:  

        all_nodes.remove(node + [
            cost_dict[(node[0], node[1])] + heucost])  
        all_nodes.append(
            node + [cost_dict[(child_node[0], child_node[1])] + cost_to_come + heucost])  #new data
        cost_dict[(node[0], node[1])] = cost_dict[(
        child_node[0], child_node[1])] + cost_to_come  #new cost
        p_id[(node[0], node[1])] = (
        child_node[0], child_node[1])  
        action_dict[(node[0], node[1])] = action
        theta_dict[(node[0], node[1])] = theta

    
    all_nodes = sorted(all_nodes, key=itemgetter(2))  
    # print all the nodes
    return all_nodes, cost_dict, p_id, action_dict, theta_dict 

no = 0

def astarturtlebot(start_point, goal_point, initial_theta):
    currentnode = [start_point[0], start_point[1]]  # Variable for storing the current node
    all_nodes = []  
    cost_dict = {(start_point[0], start_point[1]): 0}  
    closed_nodes = []
    p_id = {}  
    action_dict = {}
    theta_dict = {(start_point[0], start_point[1]): initial_theta}
    n = 0
    
    while ((currentnode[0] - goal_point[0]) ** 2 + (currentnode[1] - goal_point[
        1]) ** 2 - 0.4 ** 2 > 0):  

        # Nodes in particular direction and it is calculation
        # m1_x and m1_y are currewnt nodes
        m1_x, m1_y, m1_theta = action(currentnode, 5, 0, theta_dict[(currentnode[0], currentnode[1])])
        if obstacle_map(m1_x, m1_y) == False and ([m1_x, m1_y] not in closed_nodes) and (
                 checkobsspacePoint([m1_x, m1_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = costupdated(currentnode, goal_point,
                                                                                [m1_x, m1_y], m1_theta, [5, 0],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        
        m2_x, m2_y, m2_theta = action(currentnode, 0, 5, theta_dict[(currentnode[0], currentnode[1])])
        if obstacle_map(m2_x, m2_y) == False and ([m2_x, m2_y] not in closed_nodes) and (
                 checkobsspacePoint([m2_x, m2_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = costupdated(currentnode, goal_point,
                                                                                [m2_x, m2_y], m2_theta, [0, 5],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        
        m3_x, m3_y, m3_theta = action(currentnode, 5, 5, theta_dict[(currentnode[0], currentnode[1])])
        if obstacle_map(m3_x, m3_y) == False and ([m3_x, m3_y] not in closed_nodes) and (
                 checkobsspacePoint([m3_x, m3_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = costupdated(currentnode, goal_point,
                                                                                [m3_x, m3_y], m3_theta, [5, 5],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        
        m4_x, m4_y, m4_theta = action(currentnode, 10, 0, theta_dict[(currentnode[0], currentnode[1])])
        if obstacle_map(m4_x, m4_y) == False and ([m4_x, m4_y] not in closed_nodes) and (
                 checkobsspacePoint([m4_x, m4_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = costupdated(currentnode, goal_point,
                                                                                [m4_x, m4_y], m4_theta, [10, 0],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        
        m5_x, m5_y, m5_theta = action(currentnode, 0, 10, theta_dict[(currentnode[0], currentnode[1])])
        if obstacle_map(m5_x, m5_y) == False and ([m5_x, m5_y] not in closed_nodes) and (
                 checkobsspacePoint([m5_x, m5_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = costupdated(currentnode, goal_point,
                                                                                [m5_x, m5_y], m5_theta, [0, 10],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        
        m6_x, m6_y, m6_theta = action(currentnode, 10, 10, theta_dict[(currentnode[0], currentnode[1])])
        if obstacle_map(m6_x, m6_y) == False and ([m6_x, m6_y] not in closed_nodes) and (
                 checkobsspacePoint([m6_x, m6_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = costupdated(currentnode, goal_point,
                                                                                [m6_x, m6_y], m6_theta, [10, 10],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        
        m7_x, m7_y, m7_theta = action(currentnode, 5, 10, theta_dict[(currentnode[0], currentnode[1])])
        if obstacle_map(m7_x, m7_y) == False and ([m7_x, m7_y] not in closed_nodes) and (
                 checkobsspacePoint([m7_x, m7_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = costupdated(currentnode, goal_point,
                                                                                [m7_x, m7_y], m7_theta, [5, 10],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        
        m8_x, m8_y, m8_theta = action(currentnode, 10, 5, theta_dict[(currentnode[0], currentnode[1])])
        if obstacle_map(m8_x, m8_y) == False and ([m8_x, m8_y] not in closed_nodes) and (
                 checkobsspacePoint([m8_x, m8_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = costupdated(currentnode, goal_point,
                                                                                [m8_x, m8_y], m8_theta, [10, 5],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        closed_nodes.append(currentnode)  

        if n % 1000 == 0:
            print(n)
            for i in closed_nodes:
                plt.scatter(i[0], i[1], color='b')
           

        
        n = n + 1
        
        # print cost 
        if all_nodes != []:  
            nextnode = all_nodes.pop(0)  # Take out the next lowest cost node
            no = 0
        else:  
            print('No path found')
            no = 1
            break
         


        
        
        currentnode = [nextnode[0], nextnode[1]] 

    print('fff-',len(closed_nodes))

    path = [(currentnode[0], currentnode[1])]  # Variable for storing the optimal path from the A* Algorithm

    while (path[-1] != (start_point[0], start_point[1])):
        path.append(p_id[path[-1]])  
        plt.scatter(path[-1][0], path[-1][1], color='r')


    path.reverse()
    path_action = []
    for i in range(1, len(path)):
        path_action.append(action_dict[(path[i][0], path[i][1])])

    return path, path_action, no

##########ALL the inputs########
strt_x = float(input('X-coordinate initial :'))
strt_y = float(input('Y-coordinate initial  :'))
initial_theta_deg = float(input("Intial orientation "))
g_x = float(input('Y-coordinate goal :'))
g_y = float(input('Y-coordinate goal :'))
path_action=[]
initial_theta = (initial_theta_deg * math.pi)/180  #converting it to radians

if obstacle_map(strt_x, strt_y) == True:
    print('Initial point is out of the obstacle space')
elif obstacle_map(g_x, g_y) == True:
    print('Goal point is out of the obstacle space')
else:
    path, path_action, no = astarturtlebot([strt_x, strt_y], [g_x, g_y],initial_theta)
    if no != 1:
        choice = input('Enter yes  or no for generating nodes- ')
        if choice == 'yes':#now a choice
            plt.show()
        print('path- ',path)
        print('action sequence- ',path_action)
        print('no of steps',len(path_action))


try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')
#time library 
import time

print ('program Started')
vrep.simxFinish(-1) 
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # connecting vrep to python at specific port
if clientID!=-1:
    print ('connection to API is successful')

#retrieve motor  handles
    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
    r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_streaming)
    
    path_speeds = path_action
    for k in path_speeds:
        time = 0
        err_code1 = 1
        err_code2 = 2
        
        while(err_code1 != 0 and err_code2 != 0):
            err_code1 = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, k[0], vrep.simx_opmode_streaming)
            print(err_code1)

            err_code2 = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, k[1], vrep.simx_opmode_streaming)
            print(err_code2)

        r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

        while(time<=2):

            r, signalValue2 = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

            time = signalValue2 - signalValue

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
   
    vrep.simxGetPingTime(clientID)

  #disconnection
    vrep.simxFinish(clientID)
else:
    print ('Check API connection')
print ('Run is completed')
