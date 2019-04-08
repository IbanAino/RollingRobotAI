import numpy as np
from Modules.OccupancyGrid import OccupancyGrid
from Modules.RpLidar import RpLidar
from Modules.Robot import Robot
import matplotlib.pyplot as plt
import time


show_animation = True


OccupancyGrid = OccupancyGrid(200, 200, 50) # (gridHigh, gridWidth, cellSize)
grid = OccupancyGrid.grid
print(grid)

Robot = Robot()
Robot.x = 5000
Robot.y = 5000

lidar = RpLidar()
lidar.runLidar()

newTime = 0

while True:
    newTime = time.time()
    
    obstaclesFromRobot = lidar.GetObstacles()
    #print(obstaclesFromRobot)
    
    # Translate obstacles positions from robot to world
    obstaclesFromWorld = np.array([[80, 70]])
        
    # passage matrix
    TrobotToWorld = np.array([[np.cos(Robot.angle), (np.sin(Robot.angle))*-1, Robot.x],
        [np.sin(Robot.angle), np.cos(Robot.angle), Robot.y],
        [0, 0, 1]])
    
    for obs in obstaclesFromRobot:
        # Translate the point from the robot to the world  
        pointFromRobot = np.array([[obs.item(0)], [obs.item(1)], [1]])
        pointFromWorld = np.dot(TrobotToWorld, pointFromRobot)
        # Add the new point to the list
        obstaclesFromWorld = np.append(obstaclesFromWorld, [[pointFromWorld[0, 0], pointFromWorld[1, 0]]], axis=0)        
        '''        
        print('obs :')
        print(pointFromRobot)
        print('T matrix:')
        print(TrobotToWorld)
        print('pointFromWorld :')
        print(pointFromWorld)
        '''
        
    #print(obstaclesFromWorld)
    
    for i in obstaclesFromWorld:
        '''
        print(i[0])
        print(i[1])
        print("---")
        '''
        
        OccupancyGrid.RaycastFillsCells(Robot.x, Robot.y, i[0], i[1])
        
        OccupancyGrid.FillCell(i[0], i[1], 255)
    
    
    if show_animation:
        plt.imshow(grid)
        plt.draw()
        plt.pause(0.001)
    
    # Time for the loop to execute in seconds
    print(time.time() - newTime)
    
    #time.sleep(1)
