#Author-Piyushkumar Bhuva
#UID-116327473
#Planning project 1



import numpy as np    #for array functions like reshaping
import math

x=input("Please type numbers separated by ',' : ") # User types the value
y=x.split(',')   #spliting the string as numbers are strings here
mat=[]
l=[]
for t in y:  #iterating over length of y 
    mat.append(int(t))


def distict_number(h): #gives distict_number number for a lsit
   number=0 
   for i in range(0,len(h)) :  
        x=len(h)-1-i  #iterating over range as mentioned
        n=math.pow(10,x)   # power 
        number+= h[i]*n  #number will be reiterated becuase of +=
   return number

sol1=[1,2,3,4,5,6,7,8,0]  #goal state
solution=distict_number(sol1) 

def exist(arr,n): # checks whether a number exists in a list or does not exist
    t=False 
    for i in arr:  #reiterating in array
        if i==n:
            t=True
            break
    return t

def inversions(arr):  # Checks for inversion to determine solvability
    n=len(arr)
    initial = 0
    for i in range(n): 
        for j in range(i + 1, n): 
            if (arr[i] > arr[j]): 
                initial += 1
  
    return initial 

m1=[]
for i in range(len(mat)):   #deletes 0 here
    if (mat[i] != 0):
        m1.append(mat[i])
        
        
def rearrange(mat):         #reaching goal state
    x=np.reshape(mat,(3,3))  #numpy allows only valid matrix dimention operation
    y=x.T
    z=np.reshape(y,(1,-1))
    y=z[0]    
    return y 

inv_c=inversions(m1)
if (inv_c % 2 == 0): # if inversions are even solution exists and prints the same
    print("Solution Exists")

    nodes=[mat]
    
    nums=[distict_number(mat)]
    t=True
    depth=0
    node_no=1
    n=[1,0,0]
    node_info=[n]
    len1=0
    count=0
    while t:
        
        len2=len(nums)
        c=len2
        depth=depth+1  #depth counts steps here
        print(depth)
        
        for pos in range(len1,len2):
            address=nodes[pos].index(0)
            
            if address>2 :                   #moving up
                list=nodes[pos].copy()
                list[address], list[address - 3] = list[address - 3], list[address]  #getting answer for move
                            
                
   
                
                if not (exist(nums,s)) :
                    node_no+=1
    
                    count=count+1
                    nums.append(s)
                    n=[node_no,pos+1,depth]
                    node_info.append(n)
                    nodes.append(list)
                    
                    
                    
                    if s==solution:
                        t=False  #assigniing boolean to start 
                        print("Solution Exist")
                        
                        break
                    
            if address<6 :                              #moving down
                list=nodes[pos].copy()
                list[address], list[address + 3] = list[address + 3], list[address]
                s=distict_number(list)
    
                
                if not (exist(nums,s)) :
                    node_no+=1
                    count=count+1
                    nums.append(s)
                    n=[node_no,pos+1,depth]
                    node_info.append(n)
                    nodes.append(list)
                    
                    if s==solution:
                        t=False
                        print("Solution Exist")
                        break
                     
            if address % 3 != 0 :                     #moving left if possible
                list=nodes[pos].copy()
                list[address], list[address - 1] = list[address - 1], list[address]
                s=distict_number(list)
    
                
                if not (exist(nums,s)) :
                    
                    node_no+=1
                    count=count+1
                    nums.append(s)
                    n=[node_no,pos+1,depth]
                    node_info.append(n)
                    nodes.append(list)
                    
                    if s==solution:
                        t=False
                        print("Solution Exist")
                        break
                    
            if address %3 != 2:              #moving right if possible        
                list=nodes[pos].copy()
                list[address], list[address + 1] = list[address + 1], list[address]
                s=distict_number(list)
    #            print(s)
                if not (exist(nums,s)) :
                    node_no+=1
                    count=count+1
                    nums.append(s)
                    n=[node_no,pos+1,depth]
                    node_info.append(n)
                    nodes.append(list)
                    
                    if s==solution:
                        t=False
                        print("Solution Exist")
                        break
    
    
        if count==0:
            print("No Solution Exist")
            t=False
            break
        len1=c
        count=0
    print(node_info)
    
    
    
    #for node in node we have to reshape using numpy function and here are these function
    #    np.reshape(node,(3,3))
    #    np.reshape(node.T,(1,-1))
    fn=node_no-1  #tarce_back using node and node_info                         
          
    path=[]
    while (fn > 0):
            path.append(nodes[fn])
            k=fn
            fn=(node_info[k][1])-1
    path.append(mat)
    reverse_path=path[::-1] #reverse slicing used to traceback from goal state to starting state 
    
    reverse=[]
    for path in reverse_path:
        reverse.append(rearrange(path))
    node1=[]
    for node in nodes:
        node1.append(rearrange(node))
        
    
    
    file=open('nodes.txt','w')    # writes a new file in w mode named nodes.txt
    for node in node1:
        file.write("\n") #new node at new line
        for i in node:
            file.write(str(i)+" ")
               
        
    file.close()
    file1=open('node_info.txt','w')   #writes a new file in w mode named node_info.txt
    for node_i in node_info:    
        file1.write("\n")
        for i in node_i:
            file1.write(str(i)+ " ")    
    file1.close()
    
    file2=open('node_path.txt','w')   #writes a new file in w mode named node_path.txt
    for path in reverse:
        file2.write("\n")
        for i in path:
            
            file2.write(str(i)+" ")    
    file2.close()
else:#if solution does not exist or inversions are odd
    print("No Solution Exists")   
    file=open('nodes.txt','w') # a new file in nodes.txt in w mode
    file.write("No Solution")  #writing 'no solution'
    file.close()   #closing nodes.txt
    file1=open('node_info.txt','w')#a new file in node_info.txt
    file1.write("No Solution")  #writing 'no solution'
    file1.close() #closing node_info.txt
    file2=open('node_path.txt','w')# a new file node_path,txt
    file2.write("No Solution")  #writing 'no solution'
    file2.close()  #clsoing nodes.txt
    




                
                
                
                
            
        
        
        
    




