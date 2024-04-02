#!/usr/bin/env python
# coding: utf-8

# In[1]:


import heapq
import cv2
import numpy as np


# In[3]:


import heapq
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])
def astar(start, goal, neighbors):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        current_cost, current_node = heapq.heappop(open_set)

        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            return path[::-1]

        for neighbor in neighbors(current_node):
            tentative_g_score = g_score[current_node] + 1  

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
    return None
def neighbors(node):
    x, y = node
    possible_moves = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]  
    return [(x, y) for x, y in possible_moves if 0 <= x < 6 and 0 <= y < 4]  
start_node = (0, 0)
goal_node = (5, 3)
path = astar(start_node, goal_node, neighbors)
if path:
    print("Path found:", path)
else:
    print("No path found.")


# In[4]:


path_vis=np.zeros((6,4))


# In[5]:


for i in range (len(path)):
    path_vis[path[i][0]][path[i][1]]=1
print(path_vis)


# In[11]:


path_viscv=np.array([[[278., 315.],
                         [273., 241.],
                         [270., 163.],
                         [269.,  89.]],

                        [[393., 306.],
                         [390., 233.],
                         [383., 149.],
                         [377.,  83.]],

                        [[509., 299.],
                         [501., 222.],
                         [495., 149.],
                         [492.,  76.]],

                        [[620., 293.],
                         [617., 217.],
                         [610., 140.],
                         [609.,  64.]],

                        [[856., 274.],
                         [852., 198.],
                         [848., 122.],
                         [839.,  45.]],

                        [[969., 262.],
                         [966., 190.],
                         [960., 112.],
                         [954.,  38.]]])
'''for i in range(6):
    for j in range(4):
        for k in range(2):
            a=int(input())
            path_viscv[i][j][k]=a
            print(path_viscv)'''
print(path_viscv)
path2=np.array([(0, 1), (1, 1), (2, 1), (3, 1), (4, 1), (5, 1), (5, 2), (5, 3)])


# In[12]:


path1 = (r"C:\Users\amsmc\OneDrive\Pictures\main project image.jpg")


# In[13]:


image = cv2.imread(path1) 


# In[14]:


window_name = 'Optimal path'


# In[15]:


color = (0,0,255) 


# In[16]:


thickness = 9


# In[17]:


for i in range(len(path2)-1):
    start_point = (int(path_viscv[path2[i][0]][path2[i][1]][0]),int(path_viscv[path2[i][0]][path2[i][1]][1]))
    end_point = (int(path_viscv[path2[i+1][0]][path2[i+1][1]][0]),int(path_viscv[path2[i+1][0]][path2[i+1][1]][1]))
    image = cv2.line(image,start_point,end_point, color, thickness)


#  

# In[18]:


cv2.imshow(window_name, image)
cv2.waitKey(0)
cv2.destroyAllwindows()


# In[16]:


print(int(path_viscv[0][0][0]))
print(path)

