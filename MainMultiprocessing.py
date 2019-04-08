import numpy as np
from Modules.OccupancyGrid import OccupancyGrid
from Modules.RpLidar import RpLidar
from Modules.Robot import Robot
import matplotlib.pyplot as plt
import time
import threading
import multiprocessing


show_animation = True


OccupancyGrid = OccupancyGrid(200, 200, 50) # (gridHigh, gridWidth, cellSize)
grid = OccupancyGrid.grid
#print(grid)

Robot = Robot()
Robot.x = 5000
Robot.y = 5000

lidar = RpLidar()
lidar.runLidar()

newTime = 0

timer = time.time()

#obstaclesFromRobot = None
#global obstaclesFromRobot

# FUNCTION
def GetLidarData():
    print("GetLidarData thread")
    global obstaclesFromRobot
    obstaclesFromRobot = lidar.GetObstacles()
    #print(obstaclesFromRobot)
    #print("-----")
    
def UpdateOccupancyGrid():
    print("OccupancyGrid thread")
    # print(obstaclesFromRobot)
    
    # Translate obstacles positions from robot to world
    obstaclesFromWorld = np.array([[80, 70]])
        
    # passage matrix
    TrobotToWorld = np.array([[np.cos(Robot.angle), (np.sin(Robot.angle))*-1, Robot.x],
        [np.sin(Robot.angle), np.cos(Robot.angle), Robot.y],
        [0, 0, 1]])
    
    # Try here if obstaclesFromRobot is defined
    
    for obs in obstaclesFromRobot:
        # Translate the point from the robot to the world  
        pointFromRobot = np.array([[obs.item(0)], [obs.item(1)], [1]])
        pointFromWorld = np.dot(TrobotToWorld, pointFromRobot)
        # Add the new point to the list
        obstaclesFromWorld = np.append(obstaclesFromWorld, [[pointFromWorld[0, 0], pointFromWorld[1, 0]]], axis=0) 
    
    for i in obstaclesFromWorld:
        OccupancyGrid.RaycastFillsCells(Robot.x, Robot.y, i[0], i[1])            
        OccupancyGrid.FillCell(i[0], i[1], 255)
    
def PlotFigure(name):
    '''
    while q.empty() is False:       
        plt.imshow(mainMap) 
        plt.draw()
        plt.pause(0.1)
    '''
    print(name)
    print("------------------------------------")

    
           
# SETUP
if __name__ == '__main__':
    
    # queue to share variables between process
    # mainMap = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    # q = multiprocessing.Queue()
    
    # Process Declarations
    #p1 = multiprocessing.Process(target=PlotFigure, args=(mainMap, q))
    print("Create process")
    pr1 = multiprocessing.Process(target=PlotFigure, args=('bob',))
    pr1.start()
    pr1.join()
    print("End process")
    
    # Threads declataitons
    t1 = threading.Thread(name='daemon', target = GetLidarData)
    t2 = threading.Thread(name='daemon', target = UpdateOccupancyGrid)
    # t3 = threading.Thread(name='daemon', target = PlotFigure)

    while True:
        newTime = time.time()
        
        if t1.isAlive() is False:
            t1 = threading.Thread(name='daemon', target = GetLidarData)
            t1.start()
            #t1.join()
        if t2.isAlive() is False:
            t2 = threading.Thread(name='daemon', target = UpdateOccupancyGrid)
            t2.start()
            #t2.join(5)
        '''
        if t3.isAlive() is False:
            t3 = threading.Thread(name='daemon', target = PlotFigure)
            t3.start()
            #t2.join(5)
        '''    
         
        # Waiting for Threads to be completed
        t1.join()
        t2.join() # Wait 5 seconds until the thread exit
        # t3.join()
        
        #p1.join()


        '''
        if show_animation:
            plt.imshow(grid)
            plt.draw()
            plt.pause(0.1)
        '''
        
        # Time for the loop to execute in seconds
        print(time.time() - newTime)
        