import numpy as np
import matplotlib.pyplot as plt
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize

h = 0
l1 = 0.310
l2 = 0.310

alpha_01_min = np.radians(0)
alpha_01_max = np.radians(0)
n_alpha_01 = 10

beta_12_min = np.radians(-110)
beta_12_max = np.radians(110)
n_beta_12 = 10

beta_23_min = np.radians(-110)
beta_23_max = np.radians(110)
n_beta_23 = 10

# The following code is used to sketch the workspace of the robot arm
joint_angles = np.ndarray((0,4)) 
xyz_endpoint = np.ndarray((0,3)) 

# Loop to build the workspace plot
for alpha in np.linspace(alpha_01_min,alpha_01_max,n_alpha_01): 
    # Set the Transformation matrix for the "base" link
    T01 = np.array([[np.cos(alpha), -np.sin(alpha), 0, 0],
                    [np.sin(alpha),  np.cos(alpha), 0, 0],
                    [0,              0,             1, l1],
                    [0,              0,             0, 1]
                    ])
    
    for beta1 in np.linspace(beta_12_min, beta_12_max,n_beta_12): 
        # Set the Transformation matrix for the "upper arm" link
        T12 = np.array([[ np.cos(beta1), 0, np.sin(beta1), 0],
                        [ 0,             1,             0, 0],
                        [-np.sin(beta1), 0, np.cos(beta1), 0],
                        [ 0,             0,             0, 1]
                        ])
        
        for beta2 in np.linspace(beta_23_min,beta_23_max,n_beta_23): 
            # Set the Transformation matrix for the "forearm" link
            T23 = np.array([[ np.cos(beta2), 0, np.sin(beta2), l2],
                            [ 0,             1,             0, 0],
                            [-np.sin(beta2), 0, np.cos(beta2), 0],
                            [ 0,             0,             0, 1]
                            ])
            
            # Local vector in the last link frame (end-effector)
            local_vector_aug = np.array([l2, 0, 0, 1])

            # Final transformation: base to end-effector
            T = T01.dot(T12).dot(T23)
            pt = T.dot(local_vector_aug)[0:3]

            # Store joint angles and endpoint
            joint_angles = np.append(joint_angles, [[alpha, beta1, beta2, 0]], axis=0)
            xyz_endpoint = np.append(xyz_endpoint, np.array(pt, ndmin=2), axis=0)

# Make a nice 3D figure of the Workspace. 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xyz_endpoint[:,0], xyz_endpoint[:,1], xyz_endpoint[:,2],
           marker='o', c=xyz_endpoint[:,2], cmap='viridis')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_title('Reachable Workspace of the 3-DOF Robot Arm')
ax.set_aspect('auto')
plt.tight_layout()
plt.show()

# The following function is used to calculate the inverse kinematics of a 2-link robot arm
def forward_kinematics(joint_angles):
    alpha, beta1, beta2 = joint_angles

    T01 = np.array([[np.cos(alpha), -np.sin(alpha), 0, 0],
                    [np.sin(alpha),  np.cos(alpha), 0, 0],
                    [0,              0,             1, h],
                    [0,              0,             0, 1]
                    ])
    
    T12 = np.array([[np.cos(beta1), 0, np.sin(beta1), 0],
                    [0,             1,             0, 0],
                    [-np.sin(beta1),0, np.cos(beta1), 0],
                    [0,             0,             0, 1]
                    ])
    
    T23 = np.array([[np.cos(beta2), 0, np.sin(beta2), l1],
                    [0,             1,             0, 0],
                    [-np.sin(beta2),0, np.cos(beta2), 0],
                    [0,             0,             0, 1]
                    ])
    
    end_point_local_aug = np.array([l2, 0, 0, 1])
    end_point_global_aug = T01 @ T12 @ T23 @ end_point_local_aug
    return end_point_global_aug[:3]

def objective_function(joint_angles, target_position):
    end_point = forward_kinematics(joint_angles)
    error = np.linalg.norm(end_point - target_position)
    return error

initial_guess = np.array([np.radians(0), np.radians(0), np.radians(0)])

target_position = np.array([0, 0.1, 0.3])  # Example target position, but must modifed because we need to use ROS2 message to get the target position

result = minimize(objective_function, initial_guess, args=(target_position),
                  bounds=[(alpha_01_min, alpha_01_max), (beta_12_min, beta_12_max), (beta_23_min, beta_23_max)],
                  method='L-BFGS-B')

if result.success:
    joint_angles = result.x
    print("Optimal joint angles (radians):", joint_angles)
    print("Optimal joint angles (degrees):", np.degrees(joint_angles))
    print("End effector position:", forward_kinematics(joint_angles))
else:
    print("Optimization failed:", result.message)