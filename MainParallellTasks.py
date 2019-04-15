import multiprocessing
import time
import threading
import numpy as np
from Modules.OccupancyGrid import OccupancyGrid
from Modules.RpLidar import RpLidar
from Modules.Robot import Robot
import matplotlib.pyplot as plt


#--------------------- VARIABLES ---------------
durationProcess1 = 0
durationProcess2 = 0
durationTaskGetLidarData = 0
durationTaskUpdateOccupancyGrid = 0
    
#--------------------- FUNCTIONS ---------------

def f(name):
    print ('hello', name)


#------------------------ PROCESS ----------------
  
def ProcessRobotManagement(q1):    
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
        durationTaskGetLidarData = time.time()
        
        print("GetLidarData thread")
        global obstaclesFromRobot
        obstaclesFromRobot = lidar.GetObstacles()
        #print(obstaclesFromRobot)
        #print("-----")
        
        print("Duration task GetLidarData:", time.time() - durationTaskGetLidarData)
    
    def UpdateOccupancyGrid():
        durationTaskUpdateOccupancyGrid = time.time()
        
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
            
        print("Duration task UpdateOccupancyGrid:", time.time() - durationTaskUpdateOccupancyGrid)
        
    
    #------ Threads declarations -------
    t1 = threading.Thread(name='daemon', target = GetLidarData)
    t2 = threading.Thread(name='daemon', target = UpdateOccupancyGrid)
    
    
    #--------- MAIN LOOP -----------
    while True:
        durationProcess1 = time.time()
        
        if t1.isAlive() is False:
            t1 = threading.Thread(name='daemon', target = GetLidarData)
            t1.start()
            #t1.join()
        if t2.isAlive() is False:
            t2 = threading.Thread(name='daemon', target = UpdateOccupancyGrid)
            t2.start()

        t1.join()
        t2.join()
        
        
        q1.put(grid)
        #testMatrix = [[1, 2, 3],[4, 5, 6],[7, 8, 9]]
        #plt.imshow(testMatrix)
        #plt.imshow(grid)
        #plt.matshow(grid, 0)
        #plt.draw()
        #plt.pause(0.1)
        print("Duration process 1:", time.time() - durationProcess1)

        
def ProcessGraphicVisualisation(q1):
    durationProcess2 = time.time()
    
    #print(q1.get())
    while True:
        durationProcess2 = time.time()
        
        plt.imshow(q1.get())
        plt.draw()
        plt.pause(0.1)
        
        print("--Graphic visualisation")
        
        print("Duration process 2:", time.time() - durationProcess2)

    
    '''
    #sharedMap [0] = 1
    sharedMap [1] = 2
    sharedMap [2] = 3
    
    while True:
        print(sharedMap[0])
        time.sleep(2)       
    '''

#--------------------- SETUP ---------------

if __name__ == '__main__':
    # Queue to share data wetween process
    q1 = multiprocessing.Queue()

    
    # Shared memory variable
    sharedMap = multiprocessing.Array('i', 3)

    p = multiprocessing.Process(target=f, args=('bob',))
    p2 = multiprocessing.Process(target=ProcessRobotManagement, args=(q1, ))
    p3 = multiprocessing.Process(target=ProcessGraphicVisualisation, args=(q1, ))
    
    p.start()
    p2.start()
    p3.start()   

    p.join()
    p2.join()
    p3.join()