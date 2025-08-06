# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib import pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from scipy.optimize import minimize

# d = 0
# l1 = 0.310
# l2 = 0.310
# l3 = 0.310

# alpha_01_min = np.radians(0)
# alpha_01_max = np.radians(0)
# n_alpha_01 = 10

# beta_12_min = np.radians(-110)
# beta_12_max = np.radians(110)
# n_beta_12 = 10

# beta_23_min = np.radians(-110)
# beta_23_max = np.radians(110)
# n_beta_23 = 10

# # The following code is used to sketch the workspace of the robot arm
# joint_angles = np.ndarray((0,3)) 
# xyz_endpoint = np.ndarray((0,3)) 

# # Loop to build the workspace plot
# for alpha in np.linspace(alpha_01_min,alpha_01_max,n_alpha_01): 
#     # Set the Transformation matrix for the "base" link
#     T01 = np.array([[ np.cos(alpha), 0, np.sin(alpha), 0],
#                     [ 0,             1,             0, d],
#                     [-np.sin(alpha), 0, np.cos(alpha), 0],
#                     [ 0,             0,             0, 1]
#                     ])
    
#     for beta1 in np.linspace(beta_12_min, beta_12_max,n_beta_12): 
#         # Set the Transformation matrix for the "upper arm" link
#         T12 = np.array([[np.cos(beta1), -np.sin(beta1), 0, 0],
#                         [np.sin(beta1),  np.cos(beta1), 0, 0],
#                         [0,              0,             1, 0],
#                         [0,              0,             0, 1]
#                         ])
        
#         for beta2 in np.linspace(beta_23_min,beta_23_max,n_beta_23): 
#             # Set the Transformation matrix for the "forearm" link
#             T23 = np.array([[np.cos(beta2), -np.sin(beta2), 0, 0],
#                             [np.sin(beta2),  np.cos(beta2), 0, l1],
#                             [0,              0,             1, 0],
#                             [0,              0,             0, 1]
#                             ])
            
#             # Local vector in the last link frame (end-effector)
#             local_vector_aug = np.array([0, l2, 0, 1])
            
#             # Final transformation: base to end-effector
#             T = T01.dot(T12).dot(T23)
#             pt = T.dot(local_vector_aug)[0:3]
            
#             # Store joint angles and endpoint
#             joint_angles = np.append(joint_angles, [[alpha, beta1, beta2]], axis=0)
#             xyz_endpoint = np.append(xyz_endpoint, np.array(pt, ndmin=2), axis=0)

# # Make a nice 3D figure of the Workspace. 
# '''
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(xyz_endpoint[:,0], xyz_endpoint[:,1], xyz_endpoint[:,2],
#            marker='o', c=xyz_endpoint[:,2], cmap='viridis')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# ax.set_title('Reachable Workspace of the 3-DOF Robot Arm')
# ax.set_aspect('auto')
# plt.tight_layout()
# plt.show()
# '''
# fig = plt.figure(figsize=(12, 8))
# ax3d = fig.add_subplot(221, projection='3d')
# sc = ax3d.scatter(xyz_endpoint[:,0], xyz_endpoint[:,1], xyz_endpoint[:,2],
#                   c=xyz_endpoint[:,2], cmap='viridis', marker='o', s=5)
# ax3d.set_xlabel('X')
# ax3d.set_ylabel('Y')
# ax3d.set_zlabel('Z')
# ax3d.set_title('3D Workspace View')
# plt.colorbar(sc, ax=ax3d, shrink=0.5, label='Z height')

# # Make front, side, and top views of the workspace
# # Sketch the top view (XY plane)
# ax_xy = fig.add_subplot(222)
# ax_xy.scatter(xyz_endpoint[:,0], xyz_endpoint[:,1], c=xyz_endpoint[:,2], cmap='viridis', s=5)
# ax_xy.set_xlabel('X')
# ax_xy.set_ylabel('Y')
# ax_xy.set_title('Top View (XY Plane)')

# # Sketch the side view (XZ plane)
# ax_xz = fig.add_subplot(223)
# ax_xz.scatter(xyz_endpoint[:,0], xyz_endpoint[:,2], c=xyz_endpoint[:,2], cmap='viridis', s=5)
# ax_xz.set_xlabel('X')
# ax_xz.set_ylabel('Z')
# ax_xz.set_title('Side View (XZ Plane)')

# # Sketch the front view (YZ plane)
# ax_yz = fig.add_subplot(224)
# ax_yz.scatter(xyz_endpoint[:,1], xyz_endpoint[:,2], c=xyz_endpoint[:,2], cmap='viridis', s=5)
# ax_yz.set_xlabel('Y')
# ax_yz.set_ylabel('Z')
# ax_yz.set_title('Front View (YZ Plane)')

# plt.tight_layout()
# plt.show()

# # The following function is used to calculate the inverse kinematics of a 2-link robot arm
# def forward_kinematics(joint_angles):
#     alpha, beta1, beta2 = joint_angles

#     T01 = np.array([[ np.cos(alpha), 0, np.sin(alpha), 0],
#                     [ 0,             1,             0, d],
#                     [-np.sin(alpha), 0, np.cos(alpha), 0],
#                     [ 0,             0,             0, 1]
#                     ])
    
#     T12 = np.array([[np.cos(beta1), -np.sin(beta1), 0, 0],
#                     [np.sin(beta1),  np.cos(beta1), 0, 0],
#                     [0,              0,             1, 0],
#                     [0,              0,             0, 1]
#                     ])
    
#     T23 = np.array([[np.cos(beta2), -np.sin(beta2), 0, 0],
#                     [np.sin(beta2),  np.cos(beta2), 0, l1],
#                     [0,              0,             1, 0],
#                     [0,              0,             0, 1]
#                     ])
    
#     end_point_local_aug = np.array([0, l2, 0, 1])
#     end_point_global_aug = T01 @ T12 @ T23 @ end_point_local_aug
#     return end_point_global_aug[:3]

# def objective_function(joint_angles, target_position):
#     end_point = forward_kinematics(joint_angles)
#     error = np.linalg.norm(end_point - target_position)
#     return error

# initial_guess = np.array([np.radians(0), np.radians(0), np.radians(0)])  # Initial guess for joint angles

# target_position = np.array([0, 0.6, 0])  # Example target position, but must modifed because we need to use ROS2 message to get the target position

# result = minimize(objective_function, initial_guess, args=(target_position),
#                   bounds=[(alpha_01_min, alpha_01_max), (beta_12_min, beta_12_max), (beta_23_min, beta_23_max)],
#                   method='L-BFGS-B')

# if result.success:
#     joint_angles = result.x
#     print("Optimal joint angles (radians):", joint_angles)
#     print("Optimal joint angles (degrees):", np.degrees(joint_angles))
#     print("End effector position:", forward_kinematics(joint_angles))
# else:
#     print("Optimization failed:", result.message)

# def ik(X, Y):
#     initial_guess = np.array([np.radians(0), np.radians(0), np.radians(0)])  # Initial guess for joint angles

#     target_position = np.array([X, Y, 0])  # Example target position, but must modifed because we need to use ROS2 message to get the target position

#     result = minimize(objective_function, initial_guess, args=(target_position),
#                     bounds=[(alpha_01_min, alpha_01_max), (beta_12_min, beta_12_max), (beta_23_min, beta_23_max)],
#                     method='L-BFGS-B')

#     if result.success:
#         joint_angles = result.x
#         print("Optimal joint angles (radians):", joint_angles)
#         print("Optimal joint angles (degrees):", np.degrees(joint_angles))
#         print("End effector position:", forward_kinematics(joint_angles))
#         return joint_angles
#     else:
#         print("Optimization failed:", result.message)


import numpy as np

import numpy as np

def calculate_angles(x, y, z, L1, L2):
    d = np.sqrt(x**2 + y**2)

    # Check reachability
    if d > (L1 + L2):
        raise ValueError("Target is out of reach.")

    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if np.abs(cos_theta2) > 1:
        raise ValueError("Numerical error: no valid solution.")

    # Elbow Down and Up
    theta2_down = np.arccos(cos_theta2)
    theta2_up = -theta2_down

    def compute_theta1(theta2):
        psi = np.arctan2(y, x)
        phi = np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
        return psi - phi

    theta1_down = compute_theta1(theta2_down)
    theta1_up = compute_theta1(theta2_up)

    # Convert to degrees and round
    return {
        "elbow_down": (
            round(np.degrees(theta1_down)),
            round(90 - np.degrees(theta2_down))
        ),
        "elbow_up": (
            round(np.degrees(theta1_up)),
            round(90 - np.degrees(theta2_up))
        )
    }

# Example usage:
# result = inverse_kinematics_2link(0.4, 0.2, 0.3, 0.3)
# print("Elbow Down:", result["elbow_down"])
# print("Elbow Up:", result["elbow_up"])


print(calculate_angles(0.31, 0.31, 0, 0.31, 0.31))  # Example usage
print(calculate_angles(0, 0.62, 0, 0.31, 0.31))  # Example usage
print(calculate_angles(0.62, 0, 0, 0.31, 0.31))  # Example usage
print(calculate_angles(-0.31, 0.31, 0, 0.31, 0.31))  # Example usage
print(calculate_angles(-0.62, 0, 0, 0.31, 0.31))  # Example usage
print(calculate_angles(0.31, -0.31, 0, 0.31, 0.31))  # Example usage
