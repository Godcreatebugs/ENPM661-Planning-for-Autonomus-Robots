# -*- coding: utf-8 -*-
"""


@author: Piyush Bhuva
"""
#importing packages
import time
#making obstacle space
def obstacle_space(x, y, radius, clearance, resolution):
    flag1=1
    flag2=1
    flag3=1
    flag4=1
    m=[41/25, 0, -37/20, 38/23, -38/7, 2/19]
    c=[-261, 0, 6101/20, -8530/23, 5830/7, -1314/19]
    c_new= []
    for j in range(6):
        if j<3:
            c_new.append(c[j]+ (radius+clearance)*(1+m[j]**2)**0.5)
        else:
            c_new.append(c[j]- (radius+clearance)*(1+m[j]**2)**0.5)
    circle = lambda x,y : ((x-190)/resolution)**2 + ((y-130)/resolution)**2 - ((15+radius+ clearance)/resolution)**2
    ellipse = lambda x,y : ((x-140)/resolution)**2/((15+radius+clearance)/resolution)**2 + ((y-120)/resolution)**2/((6+radius+clearance)/resolution)**2 - 1
  ##lines to create polygon and square
    l1 = lambda x,y : x - (50 - radius- clearance)/resolution
    l2 = lambda x,y : y - (112.5+ radius+ clearance)/resolution
    l3 = lambda x,y : x - (100+ radius+ clearance)/resolution
    l4 = lambda x,y : y - (67.5 - radius- clearance)/resolution
    l5 = lambda x,y : y + m[0]*x + c[0]/resolution
    l6 = lambda x,y : y - (15 - radius- clearance)/resolution
    l7 = lambda x,y : y + m[2]*x + c[2]/resolution 
    l8 = lambda x,y : y + m[3]*x + c_new[3]/resolution 
    l9 = lambda x,y : y + m[4]*x + c_new[4]/resolution
    l10 = lambda x,y : y + m[5]*x + c_new[5]/resolution
    
    if(circle(x,y)<=0):#valid circle
        flag1=0    
    if (ellipse(x,y)<=0):#valid ellipse
        flag2=0
    if(l1(x,y)>=0 and l2(x,y)<=0 and l3(x,y)<=0 and l4(x,y)>=0):
        flag3 = 0
    if((l5(x,y)>=0 and l6(x,y)>=0 and l7(x,y)>=0 and l8(x,y)<=0) and (l9(x,y)<=0 or l10(x,y)<=0)):
        flag4 = 0
    
    if(flag1==0 or flag2==0 or flag3==0 or flag4==0):
        return False
    else:
        return True


def Obstacle_map(radius, clearance, resolution):
    st=time.time()
    all_nodes = []
    for x in range(int(250/resolution)):
        for y in range(int(150/resolution)):
            if(obstacle_space(x,y,radius,clearance,resolution)==True ):
                all_nodes.append([x, y])
    et=time.time()
    print(et-st)
   
  
#Obstacle_map(0,0,1)
    def neighbors(node):#moves around neighbour
        dirs = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [-1, -1], [1, -1], [-1, 1]]
        result = []
        weight=[]
        i=0
        for dir in dirs:
            neighbor = [node[0] + dir[0], node[1] + dir[1]]
            if i>3:
                cost=2**0.5
            else:
                cost=1  #calculating the cost
            if neighbor in all_nodes:
                result.append(tuple(neighbor))
                weight.append(cost)
            i=i+1
            
        dictionary = dict(zip(result, weight))
            #return result, weight
            #print(new_dict)
        return dictionary
    graph={}
    st1=time.time()
    print("  " + str(st1-st))
        
        
    for node in all_nodes:
        #graph = collections.OrderedDict()
        graph.update({tuple(node): neighbors(node)})
    #graph= dict(zip(tuple(all_nodes,neighbors)))
    st2=time.time()
    print("Time to load graph: "+ str(st2-st))
        #print(graph)
    
    
    file1=open('graph.txt','w')    
    file1.write(str(graph))    
    file1.close()
    return graph
    #graph = dict(zip(all_nodes, for tuple(node) in all_nodes: neighbors(node)))
    #print(neighbors([1,1]))
    
#print(Obstacle_map( 0, 0, 1))
