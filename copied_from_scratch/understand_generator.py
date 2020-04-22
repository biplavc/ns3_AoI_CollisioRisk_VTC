import numpy as np
import random
iFrames = 10
iNumVehicles=100
iAvgSpeed=60
iSpeedDist=5
iAccelDist=15
iVehTypePoissonNum=4

o =[[frame, i,np.random.poisson(iVehTypePoissonNum), random.randint(0,iNumVehicles), np.random.normal(iAvgSpeed,iSpeedDist),np.random.normal(0,iAccelDist)] for frame in range(1,iFrames) for i in range(0,iNumVehicles) ] 
print (len(o))