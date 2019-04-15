

import numpy as np
from bresenham import bresenham


class OccupancyGrid:
    
    #Constructor
    def __init__(self, gridHigh, gridWidth, cellSize):
        self._cellSize = cellSize
        self.gridWidth = gridWidth
        self.gridHigh = gridHigh
        
        self.grid = np.array([[0.0 for i in range(gridWidth)] for i in range(gridHigh)])
        #self.grid = np.empty([gridHigh, gridHigh])
        #self.grid = [[0]*gridWidth]*gridHigh
    
    # Fill the cell where a point P(x,y) is situated
    def FillCell(self, x, y, value):
        _cellX = x//self._cellSize
        _cellY = y//self._cellSize
        
        if((self.gridHigh > _cellX) & (self.gridWidth > _cellY)):
            self.grid[int(_cellX), int(_cellY)] = 255
            
            #self.grid[_cellY][_cellX] = 255
            '''
            print(x)
            print(_cellX)
            print(y)
            print(_cellY)
            print('---')
            '''
            
    def RaycastFillsCells(self, robotX, robotY, pX, pY):
        cells = tuple(bresenham(robotX, robotY, int(pX), int(pY)))
        
        for p in cells:
            pX = p[0]/self._cellSize
            pY = p[1]/self._cellSize
            #print(pX)
            #print(pY)
            if((self.gridHigh > pX) & (self.gridWidth > pY)):
                self.grid[int(pX), int(pY)] = 125

        
    
    