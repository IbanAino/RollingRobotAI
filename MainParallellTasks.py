import multiprocessing
import time
import threading
import numpy as np
from Modules.OccupancyGrid import OccupancyGrid
from Modules.RpLidar import RpLidar
from Modules.Robot import Robot
import matplotlib.pyplot as plt


#--------------------- VARIABLES ---------------

    
    
#--------------------- FUNCTIONS ---------------

def f(name):
    print ('hello', name)


#------------------------ PROCESS ----------------
  
def ProcessRobotManagement(sharedMap):
    #------ Objects declarations -------
    global OccupancyGrid
    global Robot
    global Lidar
    
    #------ Objects instanciations ------
    occupancyGrid = OccupancyGrid(200, 200, 50) # (gridHigh, gridWidth, cellSize)
    Robot = Robot()
    Robot.x = 5000
    Robot.y = 5000
    
    lidar = RpLidar()
    lidar.runLidar()
    
    #------- Variables --------
    show_animation = True
    grid = occupancyGrid.grid
    newTime = 0
    timer = time.time()
    
    #------- FUNCTIONS -------------
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
            occupancyGrid.RaycastFillsCells(Robot.x, Robot.y, i[0], i[1])            
            occupancyGrid.FillCell(i[0], i[1], 255)
            #print(i)
    
    
    #------ Threads declarations -------
    t1 = threading.Thread(name='daemon', target = GetLidarData)
    t2 = threading.Thread(name='daemon', target = UpdateOccupancyGrid)
    
    
    #--------- MAIN LOOP -----------
    while True:
        if t1.isAlive() is False:
            t1 = threading.Thread(name='daemon', target = GetLidarData)
            t1.start()
            #t1.join()
        if t2.isAlive() is False:
            t2 = threading.Thread(name='daemon', target = UpdateOccupancyGrid)
            t2.start()

        t1.join()
        t2.join()
        
        
        plt.imshow(grid)
        plt.draw()
        plt.pause(0.1)
        
        sharedMap[0] = 42
        

        
def GraphicVisualisation(sharedMap):
    
    #sharedMap [0] = 1
    sharedMap [1] = 2
    sharedMap [2] = 3
    
    while True:
        print(sharedMap[0])
        time.sleep(2)       


#--------------------- SETUP ---------------

if __name__ == '__main__':
    # Queue to share data wetween process
    q1 = multiprocessing.Queue()

    
    # Shared memory variable
    sharedMap = multiprocessing.Array('i', 3)

    
    p = multiprocessing.Process(target=f, args=('bob',))
    p2 = multiprocessing.Process(target=ProcessRobotManagement, args=(sharedMap, ))
    p3 = multiprocessing.Process(target=GraphicVisualisation, args=(sharedMap, ))
    
    p.start()
    p2.start()
    p3.start()
    

    
    p2.join()
    p.join()
    p3.join()