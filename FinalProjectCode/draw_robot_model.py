# -*- coding: utf-8 -*-
"""
Created on Tue Nov 16 23:07:39 2021

@author: Peter
"""

# Robot Model with Graphics

import numpy as np
from matplotlib import pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
from matplotlib import patches
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, pathpatch_2d_to_3d

# NOTE: scipy.spatial.transform.Rotation is a shortcut for using all the different ways of representing Rotations. 
from scipy.spatial.transform import Rotation as Rot

# Robot Parameters
h1 = 0.03; c1 = 0.04  # height (h1) and x-distance (c1) from Origin to the "shoulder" on the "base" (body1) box  
lx2=.150; ly2=0.010; lz2 = 0.020; # box size for upper arm link (body 2) - Joints are at the centers of the end faces 
lx3=.080; ly3=0.010; lz3 = 0.020; # box size for forearm link (body 3) - Joints/endpoint are at the centers of the end face

# Specify configuration to draw and compute
alpha0 = -15; beta1 = -40; beta2 = 100



def transform(rotvec,trans):
    tform = np.row_stack( (np.column_stack( (Rot.from_rotvec(rotvec).as_matrix(), trans) ), [0,0,0,1]) )
    return tform

def box(ax, lx,ly,lz,tform):
    box_nominal = np.array([[0,0,0],[0,1,0],[0,1,1],[0,0,1],[0,0,0],[1,0,0],[1,1,0],[0,1,0],[1,1,0],[1,1,1],[0,1,1],[1,1,1],[1,0,1],[0,0,1],[1,0,1],[1,0,0]])
    box_nominal_augmented = np.column_stack( (box_nominal,np.ones(box_nominal[:,1].shape) ) )
    box_scaled_augmented = (box_nominal_augmented.dot(np.diag([lx,ly,lz,1])).dot(tform.transpose()))
    box_scaled = box_scaled_augmented[:,0:3]
    hbox = ax.plot(box_scaled[:,0],box_scaled[:,1],box_scaled[:,2])
    return hbox


# def boxpatch0(ax, lx, ly, lz, tform): 
#     box_nominal = [ [[0,0,0],[0,1,0],[0,1,1],[0,0,1]] , [[0,0,0],[1,0,0],[1,1,0],[0,1,0]] , [[0,0,0],[1,0,0],[1,0,1],[0,0,1]], [[0,0,1],[1,0,1],[1,1,1],[0,1,1]], [[0,1,1],[1,1,1],[1,1,0],[0,1,0]], [[1,0,0],[1,1,0],[1,1,1],[1,0,1]] ] 
#     box_nominal_augmented = [np.column_stack( (face,np.ones((4,1)) ) ) for face in box_nominal]
#     box_scaled_augmented = [face.dot(np.diag([lx,ly,lz,1])).dot(tform.transpose()) for face in box_nominal_augmented]
#     box_scaled = [face[:,0:3] for face in box_scaled_augmented]
#     faces = Poly3DCollection(box_scaled)
#     faces.set_color((.6,.6,.6,.8))
#     faces.set_edgecolor('k')
#     hbox = ax.add_collection3d(faces)
#     return hbox

def boxpatch(ax, size=[1,1,1], shift=[0,0,0], tform=np.eye(4), color=(.6,.6,.6,1)): 
    # size is the [lx, ly, lz] length measurements of the box, in a list. 
    # shift is the [dx, dy, dz] distance to shift the box's center away from the origin, after it has been scaled. 
    box_nominal = [ [[0,0,0],[0,1,0],[0,1,1],[0,0,1]] , [[0,0,0],[1,0,0],[1,1,0],[0,1,0]] , [[0,0,0],[1,0,0],[1,0,1],[0,0,1]], [[0,0,1],[1,0,1],[1,1,1],[0,1,1]], [[0,1,1],[1,1,1],[1,1,0],[0,1,0]], [[1,0,0],[1,1,0],[1,1,1],[1,0,1]] ] 
    box_centered = np.array(box_nominal) - 0.5
    box_sized = [face.dot(np.diag(size)) for face in box_centered]
    # box_shifted = box_sized + np.array([[list(shift)]*4]*6)  # this uses list cuplication by the * operator
    box_shifted = [ [vertex + shift for vertex in face] for face in box_sized]
    box_shifted_augmented = [np.column_stack( (face,np.ones((4,1)) ) ) for face in box_shifted]
    box_transformed_augmented = [face.dot(tform.transpose()) for face in box_shifted_augmented]
    box_transformed = [face[:,0:3] for face in box_transformed_augmented]
    faces = Poly3DCollection(box_transformed,facecolor=color,edgecolor='k')
    hbox = ax.add_collection3d(faces)
    return hbox

# def cylinder(ax, R,h): 
    

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z');
    # ax.set_aspect('equal')
    plt.title('Robot model')
    ax.set_xlim([-0.2,0.4])
    ax.set_ylim([-0.3,0.3])
    ax.set_zlim([-0.2,0.4])
    ax.set_box_aspect([1,1,1])
    ax.set_proj_type('ortho')  # Orthographic projection, as opposed to "persp" = Perspective
    
   
    ## Add a circle under the robot to show the ground plane
    # boxpatch(ax, size=[.6,.6,0], shift=[0,0,0], tform = np.eye(4), color=(.9,.9,.9,1))    
    # ax.add_collection3d(Poly3DCollection([[[-1,-1,0],[-1,1,0],[1,1,0],[1,-1,0]]]))
    ptch = patches.CirclePolygon((0,0), radius=0.3,facecolor=(.9,.9,.9,.9),edgecolor='k')
    ax.add_patch(ptch)
    pathpatch_2d_to_3d(ptch,z=0,zdir='z')

    # Compute the Transforms for the segments of the robot
    tform01 = transform(np.radians([0,0,alpha0]), [0,0,0]) 
    tform12 = transform(np.radians([0,beta1,0]), [c1,0,h1])
    tform23 = transform(np.radians([0,beta2,0]), [lx2, 0,0])
    
    # boxpatch0(ax, 0.03,0.03,0.04,tform01)
    # boxpatch0(ax, lx, ly, lz, tform01.dot(tform12))
    # boxpatch0(ax, lx, ly, lz, tform01.dot(tform12).dot(tform23))
    
    # Now add the various robot links. 
    boxpatch(ax, size=[2*c1, 2*c1, h1], shift=[0,0,h1/2], tform = tform01)
    boxpatch(ax, size=[lx2,ly2,lz2], shift=[lx2/2,0,0], tform = tform01.dot(tform12))
    boxpatch(ax, size=[lx3,ly3,lz3], shift=[lx3/2,0,0], tform = tform01.dot(tform12).dot(tform23))
    
    # Now draw a point representing the endpoint of the robot
    p_augmented = tform01.dot(tform12).dot(tform23).dot([lx3,0,0,1])
    p = p_augmented[0:3]
    
    # Draw the Origin and Endpoint, and a line for the axes
    ax.scatter([0,p[0]],[0,p[1]],[0,p[2]],marker='o',color='r',facecolor='r')
    plt.plot([0,0,0.3,0,0],[0.3,0,0,0,0],[0,0,0,0,0.3],'--k')
    
    print('Endpoint: {}'.format(p))
    plt.title('Robot Model: Endpoint {}'.format(p))
    
    plt.show()

    # Solve the Inverse Kinematics to check the calculations:
    alpha = np.arctan2(p[1],p[0])
    R = np.sqrt(p[0:2].dot(p[0:2])) 
    dR = R - c1 
    dZ = p[2] - h1
    
    d = np.sqrt(dR**2+dZ**2)
    psi = np.arctan2(-dZ,dR)
    phi = np.arccos( (lx3**2 - lx2**2 - d**2)/(-2*lx2*d) )
    beta2mag = np.arctan2(d*np.sin(phi) , d*np.cos(phi)-lx2)
    
    alpha0a = alpha
    beta1a = psi - phi
    beta2a = beta2mag
    
    alpha0b = alpha
    beta1b = psi+phi
    beta2b = -beta2mag
    
    print('Intermediate Variables - Length (m): dR = {:.4f}, dZ = {:.4f}, d = {:.4f}'.format(dR,dZ,d))
    print('Intermediate Variables - Angle (rad): psi = {:.4f}, phi = {:.4f}, beta2magnitude = {:.4f}'.format(psi,phi,beta2mag) )
    
    print('solution A angles (rad): {0}'.format(np.degrees([alpha0a, beta1a, beta2a])))
    print('solution B angles (rad): {0}'.format(np.degrees([alpha0b, beta1b, beta2b])))
                                                         
    


if __name__ == '__main__':
    main()
