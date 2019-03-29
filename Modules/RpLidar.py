from rplidar import RPLidar
import numpy as np
import math


class RpLidar:
    
    # Constructor
    def __init__(self):
        
        # Initialize Lidar Lidar
        self.lidar = RPLidar('com18')
        info = self.lidar.get_info()
        print(info)
        health = self.lidar.get_health()
        print(health)
        
        
    def runLidar(self):
        self.iterator = self.lidar.iter_scans(max_buf_meas=1200)
        
        
    def stopLidar(self):
        self.lidar.stop()
        self.lidar.disconnect()
        
        
    def GetObstacles(self):
        
        scan = next(self.iterator)
        
        obstacles = np.array([[80, 70]])
    
        for y in scan:
            #print(y)
            
            # STEP 1: extract polar coordonates from scan
            pPolar = [math.radians(y[1]), y[2]] # [angle in radian, distance]            
            #print('Polar :')
            #print(pPolar)
                        
            # STEP 2: compute cartesian coordonates
            angle = pPolar[0]
            passageMatrix = np.array([[np.cos(angle), np.sin(angle)],[(np.sin(angle))*-1, np.cos(angle)]])
            distanceMatrix = np.array([[pPolar[1]], [0]])
            pCart = np.dot(passageMatrix,distanceMatrix)
            #print('Cartesian :')
            #print(pCart)
            
            # STEP 4: save the obstacles inside an array
            obstacles = np.append(obstacles, [[pCart[0, 0], pCart[1, 0]]], axis = 0)

        return obstacles
            
            
            
            