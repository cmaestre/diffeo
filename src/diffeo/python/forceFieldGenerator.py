#!/usr/bin/env python
from __future__ import print_function

import numpy as np
from diffeoPy import *
import time
import os

import xml.etree.ElementTree as ET

def diffeoForceFieldGeneration(target, scalingVec):
    os.chdir('/home/maestre/git/diffeo/src/diffeo/python')
    
    target = target.T

    dim, nPoints = target.shape
    timeVec = np.linspace(1., 0., nPoints, endpoint=True)
    
    #Get the path options
    parsXML = ET.parse('../parameters/diffeoPars.xml')
    root = parsXML.getroot()
    
    inputPath = root.find('generalOpts').find('inputPath').get('value')
    resultPath = root.find('generalOpts').find('resultPath').get('value')
    targetName = root.find('generalOpts').find('targetName').get('value')
    scalingName = root.find('searchOpts').find('distanceScaling').get('value')
    timeName = root.find('generalOpts').find('targetTime').get('value')
    
    assert not( (inputPath is None) or (resultPath is None) or (targetName is None) or (timeName is None) ), "At least one path definition not specified or accessible in xml"
    
    #Create the folder if non existing 
    subprocess.call(["mkdir", "-p", inputPath])
    subprocess.call(["mkdir", "-p", resultPath])
    
    #Save the data in cpp format at the specified place
    #And save the time file
    #def Array2TXT(fileName, aArray, fileType="cpp", format="%.18e")
    Array2TXT(inputPath+targetName, target)
    Array2TXT(inputPath+timeName, timeVec)
    if not scalingName == 'manual':
        Array2TXT(inputPath+scalingName, scalingVec)
    
    #Get the diffeo move obj
    thisMovement = PyDiffeoMoveObj()
    
#    xCpp = TXT2Matrix(resultPath+"sourceTransform")   
#    #Get stuff to perform integration
#    #The demonstration takes 1sec to complete we will integrate for 1.5sec
#    tFinal = 1.1
#    deltaT = 1e-3
#    
#    allT = np.arange(0., tFinal, deltaT)
#    nPoint = allT.size
#    allX = np.zeros((dim, nPoint), order='fortran')
#    #Create an offset that will occur at about 0.5 sec
#    xOffset = np.array([-0.01, 0.01])
#    nChange = np.floor(nPoint/2)
#    
#    #Initialize the point
#    print(target.flags)
#    xCurr = np.asfortranarray(target[:,0], dtype=np.float64).copy()
##    print(target[:,0])
#    print(xCurr.flags)
#    #Dummy var for velocity
#    vCurr = np.zeros(dim, dtype=np.float64, order='fortran')
#    
##    pFig, pAx = plt.subplots(1,1)
#    
#    T = time.time()
#    for k in range(nPoint):
#        print(k)
#        allX[:,k]=xCurr
#        #pAx.plot(xCurr[0], xCurr[1], 'x', color='grey')
#        #getVelocity(self, np.ndarray[np.float64_t, ndim=1, mode = 'c'] xIn, np.ndarray[np.float64_t, ndim=1, mode = 'c'] vOut=np.zeros(0), whichSpace = 0):
#        thisMovement.getVelocity(xCurr, vCurr) #Important for you: Returns the desired velocity given a point (in the defined space)
#        print('For point {0} with coords {1} the velocity is {2}'.format(k, xCurr, vCurr))
#        xCurr += vCurr*deltaT #explicit forward euler
#        if k == nChange:
#            thisMovement.setNewTranslation(xOffset)
#    T = time.time()-T
#    print("It took {0} seconds to perform simulations".format(T))
#    
#    pAx.plot( target[0,:], target[1,:], '.-r' ) ## orig
#    pAx.plot( target[0,:]+xOffset[0], target[1,:]+xOffset[1], '--', color='orange' ) ## orig + offset
#    pAx.plot( xCpp[0,:], xCpp[1,:], '.-b', linewidth=1 )
#    pAx.plot( allX[0,:], allX[1,:], '-g', linewidth=5 ) #The actually followed traj
#    plt.show()
#    return xCpp.T
    
    
    ## creqte the grid
    grid_dim = np.linspace(0,5,16)
    a, b = np.meshgrid(grid_dim, grid_dim)
    grid_pos_vector = np.asfortranarray(np.vstack((a.flatten(), b.flatten())))
#    grid_orien_vector = np.zeros(grid_pos_vector.shape, dtype=np.float64, order='fortran')
#    print(grid_pos_vector.flags)
#    print(grid_orien_vector.flags)    
#    print('Coords:', grid_pos_vector)
#    print('Velocities: ', grid_orien_vector)
    
       
    ## compute velocities for each position of the grid
    grid_vel_values = []
    for pos in range(len(grid_pos_vector[0])):
        xCurr = np.asfortranarray([grid_pos_vector[0][pos], grid_pos_vector[1][pos]], 
                                  dtype=np.float64).copy()
#        if xCurr not in target:
        vCurr = np.zeros(len(xCurr), dtype=np.float64, order='fortran')
        thisMovement.getVelocity(xCurr, vCurr)
        tmp = np.linalg.norm(vCurr)
        vCurrNorm = vCurr/tmp
        grid_vel_values.append([xCurr, vCurrNorm ])
        print('For point {0} the velocity is {1}'.format(
              xCurr, vCurrNorm))        

    ## print velocities
    pFig, pAx = plt.subplots(1,1) 
    pAx.plot( target[0,:], target[1,:], '.-b', linewidth = 2)
    for pos in range(len(grid_vel_values)):
        curr_pos = grid_vel_values[pos][0]
        curr_vel = grid_vel_values[pos][1]
        pAx.plot( [curr_pos[0]], [curr_pos[1]], 'o', 
                 color = 'black', markersize=2)
        pylab.arrow(curr_pos[0], curr_pos[1], 
                    curr_vel[0], curr_vel[1], 
                    fc="k", ec="k",
                    head_width=0.05, head_length=0.1,
                    edgecolor='grey', linestyle='dotted')
    plt.show()
    
#    plt.close()
        
    return grid_vel_values

if __name__ == "__main__":

#    target = np.array([[0.654974, -0.0959728, 0], 
#                        [0.655759, -0.0540905, 0],
#                        [0.655, -0.001, 0],
#                        [0.605, -0.001, 0],
#                        [0.555, -0.001, 0],
#                        [0.505, 0.049, 0],
#                        [0.555, 0.049, 0],
#                        [0.605, 0.099, 0],
#                        [0.655, 0.149, 0]],
#                         np.float64)
#    scalingVec = np.array([1., 1., 0.1, 0.05, 0.05, 0.05, 0.1, 1., 1.],
#                          np.float64)

#    target = np.array([[0, 0], 
#                        [1,1],
#                        [1,3],
#                        [2,3],
#                        [3,3],
#                        [3,4],
#                        [4,4]],
#                         np.float64)

    target = np.array([[4, 1], 
                        [2,1],
                        [1,3],
                        [2,4],
                        [4,4]],
                         np.float64)


    scalingVec = np.ones(len(target),np.float64)

    grid_vel_values = diffeoForceFieldGeneration(target, scalingVec)
#    print(grid_vel_values)
























