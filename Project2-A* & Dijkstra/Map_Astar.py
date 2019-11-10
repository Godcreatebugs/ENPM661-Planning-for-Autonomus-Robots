# -*- coding: utf-8 -*-
"""


@author: Piyush Bhuva
"""
#importing packages

"""
In the homework given I have used a two point form of line and here 
I tried to make a list of slopes and intercepts for the same(ease of slicing and indexing).
Here I used slop-intercept form of line. It was comapritivly easy and ellegent way
to implement and setting flag was to check all 4 shapes are in obstacle space"""

def obstacle_space(x, y, radius, clearance, resolution):
    #setting flags
    flag1=1
    flag2=1
    flag3=1
    flag4=1
    m=[41/25, 0, -37/20, 38/23, -38/7, 2/19] #slops for lines
    c=[-261, 0, 6101/20, -8530/23, 5830/7, -1314/19] #intercepts for line
    c_new= []
    for j in range(6):
        if j<3:##for lines attaching y-intercept and slopes too
            c_new.append(c[j]+ (radius+clearance)*(1+m[j]**2)**0.5)
        else:
            c_new.append(c[j]- (radius+clearance)*(1+m[j]**2)**0.5)
    circle = lambda x,y : ((x-190)/resolution)**2 + ((y-130)/resolution)**2 - ((15+radius+ clearance)/resolution)**2
    ellipse = lambda x,y : ((x-140)/resolution)**2/((15+radius+clearance)/resolution)**2 + ((y-120)/resolution)**2/((6+radius+clearance)/resolution)**2 - 1
    l1 = lambda x,y : x - (50 - radius- clearance)/resolution
    l2 = lambda x,y : y - (112.5+ radius+ clearance)/resolution
    l3 = lambda x,y : x - (100+ radius+ clearance)/resolution
    l4 = lambda x,y : y - (67.5 - radius- clearance)/resolution
    l5 = lambda x,y : y + m[0]*x + c[0]/resolution
    l6 = lambda x,y : y - (15 - radius- clearance)/resolution
    l7 = lambda x,y : y + m[2]*x + c_new[2]/resolution 
    l8 = lambda x,y : y + m[3]*x + c_new[3]/resolution 
    l9 = lambda x,y : y + m[4]*x + c_new[4]/resolution
    l10 = lambda x,y : y + m[5]*x + c_new[5]/resolution
   ##flages to check for availibility of shapes into mobstacle map
    if(circle(x,y)<=0):
        flag1=0    
    if (ellipse(x,y)<=0):
        flag2=0
    if(l1(x,y)>=0 and l2(x,y)<=0 and l3(x,y)<=0 and l4(x,y)>=0):
        flag3 = 0
    if((l5(x,y)>=0 and l6(x,y)>=0 and l7(x,y)>=0 and l8(x,y)<=0) and (l9(x,y)<=0 or l10(x,y)<=0)):
        flag4 = 0
    
    if(flag1==0 or flag2==0 or flag3==0 or flag4==0):
        return False
    else:
        return True
#if obstacle space is correct append the touple and make a list 
def elements(radius,clearance,resolution): 
    all_nodes=[]
    for x in range(int(250/resolution)):
            for y in range(int(150/resolution)):
                if(obstacle_space(x,y,radius,clearance,resolution)==True ): #calling obstacle space from above
                    all_nodes.append(tuple([x, y])) #make it a touple
                    
    return set(all_nodes)
#elemnt= elements(0,0,0.5)
     #m=[41/25, 0, -37/20, 38/23, -38/7, 2/19] #slops for lines
    #c=[-261, 0, 6101/20, -8530/23, 5830/7, -1314/19]

     