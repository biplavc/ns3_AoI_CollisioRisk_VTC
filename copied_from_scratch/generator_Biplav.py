#!/usr/bin/python3

# Author: Avik Dayal, modified by Biplav
# Script to generate vehicle simulation data

import sys
import numpy as np
from random import randint

iFrames = 10
iNumVehicles=100
iAvgSpeed=60
iSpeedDist=5
iAccelDist=15
iVehTypePoissonNum=4
argc= len(sys.argv)
if argc > 1:
   iFrames = int(sys.argv[1])
if argc > 2:
   iNumVehicles = int(sys.argv[2])
if argc > 3:
   iAvgSpeed = int(sys.argv[3])
if argc > 4:
   iSpeedDist = int(sys.argv[4])
if argc > 5:
   iAccelDist = int(sys.argv[5])
if argc > 6:
   iVehTypePoissonNum = int(argv[6])


print(iFrames, iNumVehicles, iAvgSpeed, iSpeedDist, iAccelDist, iVehTypePoissonNum)

print ("Generating vehicle simulation data", file=sys.stderr)
print ("----------------------------------", file=sys.stderr)
print ("Number of frames   =", iFrames,      file=sys.stderr)
print ("Number of Vehicles =", iNumVehicles, file=sys.stderr)
print ("Average speed      =", iAvgSpeed ,   file=sys.stderr)
print ("Speed distribution =", iSpeedDist,   file=sys.stderr)
print ("Accel distribution =", iAccelDist,   file=sys.stderr)
print ("Vehicle Poisson    =", iVehTypePoissonNum,  file=sys.stderr)
print ("----------------------------------", file=sys.stderr)
print ("Usage: ./generate <frames> <vehicles> <speed> <speed-dist> <accel-dist> <veh-pois>", file=sys.stderr)
print ("----------------------------------", file=sys.stderr)

# generating data 
o =[[frame, i,np.random.poisson(iVehTypePoissonNum), randint(0,iNumVehicles), np.random.normal(iAvgSpeed,iSpeedDist),np.random.normal(0,iAccelDist)] for frame in range(1,iFrames) for i in range(0,iNumVehicles)] 

f = open('vehicle.csv','w')
sys.stdout = f
print('\n'.join([' '.join(['{0:.0f}'.format(item) for item in row]) for row in o]))
print ("Created file : vehicle.csv", file=sys.stderr)

