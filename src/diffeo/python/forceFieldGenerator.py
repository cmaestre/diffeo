#!/usr/bin/env python
from __future__ import print_function

import numpy as np
from diffeoPy import *
import time
import os

import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pylab

'''
a
'''
def plot_setup(obj_pos = None):
    
    fig = plt.figure(figsize=(10,10))
    fig.clf()
#    ax = Axes3D(fig)
    ax = fig.gca(projection='3d')

    # labels
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    
#    ## view
#    ## All commented = diagonal view
#    if pg.plot_angle == 'top':
#        ax.view_init(90,-90) # top view
#    elif pg.plot_angle == 'front':
#        ax.view_init(0,0) # front view
#    elif pg.plot_angle == 'side':
#        ax.view_init(0,270) # left view
#    ax.view_init(90,-90)

    ## object
    ax.bar3d(obj_pos[0] - .05/2, 
             obj_pos[1] - .05/2, 
             obj_pos[2] - .05/2, 
             [.05], [.05], [.05], 
             color='green',
             alpha=0.2,
             edgecolor='none')

    # robot
    robot_width = .2
    robot_height = .6
    robot_length = .4
    ax.bar3d(-robot_width/2, 
             -robot_length/2, 
             -robot_height/2, 
             robot_width, robot_length, robot_height, 
             color='red',
             alpha=0.2,
             edgecolor='none')
             
    # limits
    if obj_pos != None:
        lim = 0.5
        ax.set_xlim3d([obj_pos[0]-lim, obj_pos[0]+lim])
        ax.set_ylim3d([obj_pos[1]-lim, obj_pos[1]+lim])
        ax.set_zlim3d([obj_pos[2]-lim, obj_pos[2]+lim])             

    return fig, ax

'''
a'
'''
''' 
Write the trajs dataset 
'''     
def write_dataset(filename,
                  force_field_wp_vector):
    
#    experiment_folder = ""
#    filename = experiment_folder + \
#                pg.initial_datasets_folder + \
#               'novelty_dataset.csv'    
        

    file = open(filename, 'w')              
    for force_field_obj in force_field_wp_vector:
        force_field = force_field_obj[0] ## [[eef_pos, eef_vel]]
        init_obj_pos = force_field_obj[1]
        final_obj_pos = force_field_obj[1]
             
        for eef_pos_vel in force_field:
            eef_pos = eef_pos_vel[0]
            eef_vel = eef_pos_vel[1]
            
            ## eef pos
            file.write(str(eef_pos[0]))
            file.write(',')
            file.write(str(eef_pos[1]))
            file.write(',')
            file.write(str(eef_pos[2]))
            file.write(',')

            ## eef rot
            file.write(str(eef_vel[0]))
            file.write(',')
            file.write(str(eef_vel[1]))
            file.write(',')
            file.write(str(eef_vel[2]))
            file.write(',')
            
        ## init obj pos
        file.write(str(init_obj_pos[0]))
        file.write(',')
        file.write(str(init_obj_pos[1]))
        file.write(',')
        file.write(str(init_obj_pos[2]))
        file.write(',')
        
        ## init final_obj_pos pos
        file.write(str(final_obj_pos[0]))
        file.write(',')
        file.write(str(final_obj_pos[1]))
        file.write(',')
        file.write(str(final_obj_pos[2]))
        file.write(',')
        
        file.write('\n') 
    file.close()

'''
Read dataset of 1 traj externally generated
and transform it into raw deltas
traj = [EP1 EV1 EP2 EV2 ... OP]
EP = x y z ## eef position
EV = x y z ## eef velocity
OP = x y z ## obj position
'''
def read_dataset_force_fields(filename):
    delta_vector = []
    lines = open(filename, 'r').readlines()
    for line in lines:        
        current_delta_vector = []
        force_field_vector = line[:-2].split(',') ## remove final , and EOL
        nb_obj = 1# len(rospy.get_param("obj_name_vector")) ## expected
        
        obj_initial_pos = [float(force_field_vector[-6]),
                           float(force_field_vector[-5]),
                           float(force_field_vector[-4])]
        obj_final_pos = [float(force_field_vector[-3]),
                         float(force_field_vector[-2]),
                         float(force_field_vector[-1])]
        obtained_effect = discr.compute_effect(obj_initial_pos,
                                                 obj_final_pos)
        
        related_info_size = 6 + 6*nb_obj
        nb_positions = force_field_vector[:-6] / related_info_size
        if not nb_positions % related_info_size:
            print("ERROR - read_dataset_force_fields : wrong nb of values in the dataset")

        for pos in range(nb_positions):
            
            current_x = float(force_field_vector[pos+0])
            current_y = float(force_field_vector[pos+1])
            current_z = float(force_field_vector[pos+2])
            
            vel_x = float(force_field_vector[pos+related_info_size+0])
            vel_y = float(force_field_vector[pos+related_info_size+1])
            vel_z = float(force_field_vector[pos+related_info_size+2])            
                
            current_delta = delta.Delta(
                obtained_effect,
                current_x,current_y,current_z,
                vel_x, vel_y,vel_z,
                obj_initial_pos,
                obj_final_pos)
         
            current_delta_vector.append(current_delta)
        delta_vector += current_delta_vector
    
    ## compute move step length
    print('Move step length:', 2) ## TODOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
    
    return delta_vector

'''
a
'''
def diffeoTrajAnalyzer(target, scalingVec):
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
    
    return thisMovement

'''
a
'''
def diffeoForceFieldGeneration(movement,
                               curr_wp,
                               vel_next_wp,
                               wp_offset,
                               nb_points_axis,
                               obj_pos,
                               target):
    
    ## create the grid around the wp
    grid_x = np.linspace(curr_wp[0] - wp_offset,
                         curr_wp[0] + wp_offset,
                         nb_points_axis)
                         
    grid_y = np.linspace(curr_wp[1] - wp_offset,
                         curr_wp[1] + wp_offset,
                         nb_points_axis)
                         
    grid_z = np.linspace(curr_wp[2] - wp_offset,
                         curr_wp[2] + wp_offset,
                         nb_points_axis)                         
    
    a, b, c = np.meshgrid(grid_x, grid_y, grid_z)
    grid_pos_vector = np.asfortranarray(np.vstack((a.flatten(), 
                                                   b.flatten(),
                                                   c.flatten())))
#    print('Coords:', grid_pos_vector)
#    print('Velocities: ', grid_orien_vector)    
       
    ## compute velocities for each position of the force field
      
    if vel_next_wp != [0,0,0]: ## traj wp added to force field
        tmp = np.linalg.norm(vel_next_wp) 
        vCurrNorm = vel_next_wp/tmp
        vCurrNorm = [round(v, 3) for v in vCurrNorm]
        force_field_values = [[curr_wp,vCurrNorm]]
    else:
        force_field_values = []
    
    grid_plot_pos = []
    grid_plot_vel = []
    for pos in range(len(grid_pos_vector[0])):
        xCurr = np.asfortranarray([grid_pos_vector[0][pos], 
                                   grid_pos_vector[1][pos],
                                   grid_pos_vector[2][pos]], 
                                   dtype=np.float64).copy()
        vCurr = np.zeros(len(xCurr), dtype=np.float64, order='fortran')
        movement.getVelocity(xCurr, vCurr)
        tmp = np.linalg.norm(vCurr)
        vCurrNorm = vCurr/tmp
        vCurrNorm = [round(v, 3) for v in vCurrNorm]
        force_field_values.append([xCurr, vCurrNorm ])
        grid_plot_pos.append(xCurr)
        grid_plot_vel.append(vCurr)
#        print('For point {0} the velocity is {1}'.format(xCurr, vCurrNorm))        

    ## print velocities
    pFig, pAx = plot_setup(obj_pos)
    pAx.plot(target[:,0], target[:,1], target[:,2],
             '.-r', linewidth = 2)
             
    pAx.quiver([v[0] for v in grid_plot_pos],
               [v[1] for v in grid_plot_pos],
               [v[2] for v in grid_plot_pos],
               [v[0] for v in grid_plot_vel],
               [v[1] for v in grid_plot_vel],
               [v[2] for v in grid_plot_vel],
               length = 0.1,
               arrow_length_ratio = .1)
    plt.show()
        
    return force_field_values


if __name__ == "__main__":
#    target = np.array([[0, 0], 
#                        [1,1],
#                        [1,3],
#                        [2,3],
#                        [3,3],
#                        [3,4],
#                        [4,4]],
#                         np.float64)

#    target = np.array([[4, 1], 
#                        [2,1],
#                        [1,3],
#                        [2,4],
#                        [4,4]],
#                         np.float64)

    target = np.array([[2, 2, 0], 
                       [2, 4, 0],
                       [4, 4, 0]],
                       np.float64)

    scalingVec = np.ones(len(target),np.float64)

    ## create a force field for each wp
    wp_offset = 1
    nb_points_axis = 5
    init_obj_pos = [4, 4, 0]

    movement = diffeoTrajAnalyzer(target,
                                  scalingVec)
    
    force_field_wp_vector = []
    pos = 0
    for pos in range(len(target)):
        curr_wp = target[pos]
        if pos != len(target)-1:
            next_wp = target[pos+1]    
            vel_next_wp = [next_wp[0] - curr_wp[0],
                           next_wp[1] - curr_wp[1],
                           next_wp[2] - curr_wp[2]]
        else:
            vel_next_wp = [0,0,0]
            
        final_obj_pos = [6, 4, 0]

        force_field = diffeoForceFieldGeneration(movement,
                                                 curr_wp,
                                                 vel_next_wp,
                                                 wp_offset,
                                                 nb_points_axis,
                                                 init_obj_pos,
                                                 target)
        
        force_field_wp_vector.append([force_field, 
                                      init_obj_pos,
                                      final_obj_pos])
        #    print(force_field_values)