import numpy as np
from types import SimpleNamespace
from matplotlib import pyplot as plt

# =============================================================================
# State Link Lengths (MuJoCo units are flexible, be it mm, cm or m)
# =============================================================================
l1 = 1
l2 = 1
l3 = 0.25


# =============================================================================
# Values for joint angles (CCW +ve)
# =============================================================================
theta1 = np.deg2rad(90) # joint1
theta2 = np.deg2rad(-90) # joint2
theta3 = np.deg2rad(-90) # joint3


# =============================================================================
# Create a math function to derive and calc Forward Kinematics
# =============================================================================
angles = np.array([theta1, theta2, theta3])
params = (l1, l2, l3) # put into a tuple since these are fixed-values that won''t be changed


def forward_kinematics(angles, params):
    # call angle values
    theta1 = angles[0]
    theta2 = angles[1]
    theta3 = angles[2]

    # call params values
    l1, l2, l3 = params

    # assign your cos and sin values for easier callback
    c1 = np.cos(theta1)
    s1 = np.sin(theta1)

    c2 = np.cos(theta2)
    s2 = np.sin(theta2)

    c3 = np.cos(theta3)
    s3 = np.sin(theta3)


    # Homogeneous transform matrix from frame 0 to 1 
    hom_0_1 = np.array([[c1, -s1, 0, 0],
                        [s1, c1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    
    # Homogeneous transform matrix from frame 1 to 2 
    hom_1_2 = np.array([[c2, -s2, 0, l1],
                        [s2, c2, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    # Homogeneous transform matrix from frame 2 to 3 
    hom_2_3 = np.array([[c3, -s3, 0, l2],
                        [s3, c3, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    

    # Homogeneous transform matrix from frame 0 to 3
    # In jupyter we use @ instead of * to make the generated output nicer to see
    hom_0_3 = hom_0_1 @ hom_1_2 @ hom_2_3


    # Homogeneous transform matrix from frame 3 to end-effector
    # Rotation Matrix becomes Identity Matrix because no rotation from frame 3 to end-effector
    hom_3_end = np.array([[1, 0, 0, 0.25],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

    # Final Homogeneous transform, multiply all the matrix frame together, from frame 0 to end-effector
    hom_0_end = hom_0_3 @ hom_3_end
    # ----------------------------------------------------
    # OR there's another shortcut way using `np.matmul()`
    # First indicate the transformation matrix from frame 3 to end-effector in this manner:
    trans_3_end = np.array([l3, 0, 0, 1]) # This isn't a (4x1) matrix, instead it has a shape (4,) - not a row or a column 
                                            # just a flat list of 4 numbers with one axis (1-Dimension)
    # Note: regular matrix multiplication (row x column) doesn't apply to something that is 1-D 

    # Second, compute the Final Homogeneous Transform, from frame 0 to EE using `np.matmul()`
    hom_0_ee = np.matmul(hom_0_3, trans_3_end) # `matmul`: if 2nd argument is 1-D, it's promoted to a matrix by appending 1 to its shape, temporarily becoming shape (4,1)
                                               # multiplication is performed, and then that appended dimension is removed from the result
                                               # In this case: (4x4) @ (4x1) = (4,), because of `matmul`, you get a flat (4,) array instead of (4,1) like you may expect
                                               # Actual numbers will be correct and identitcal between (4,) and (4,1)
    # ----------------------------------------------------           

    # print out both methods for your viewing
    # print('Method 1: Usual Way')
    # print(hom_0_end)
    # print(hom_0_end.shape)
    # print('\n')
    # print('Method 2: `matmul` way')
    # print(hom_0_ee)
    # print(hom_0_ee.shape)                                 

    # ----------------------------------------------------  

    # Planar [2D], if you're just focused on the x and y coords of the end-effector
    # using the results from `matmul`
    # end_xy = np.array([hom_0_ee[0], hom_0_ee[1]])
    # print('\nOnly x and y:')
    # print(end_xy)
    # print(end_xy.shape)

    # ---------------------------------------------------- 

    # Plot for Animations, refer to iPad drawings but essentially it's just Body Frames 1, 2, 3 and EE
    # For ease, I'll refer to them respectively as point1, point2, point3 and pointend
    # For the math behind this, refer to notes
    # Optional: Adjust the result to only give x & y coords

    point1 = hom_0_1 @ np.array([0, 0, 0, 1])
    point1_xy = np.array([point1[0],point1[1]])

    point2 = (hom_0_1 @ hom_1_2) @ np.array([0, 0, 0, 1])
    point2_xy = np.array([point2[0],point2[1]])

    point3 = (hom_0_1 @ hom_1_2 @ hom_2_3) @ np.array([0, 0, 0, 1])
    point3_xy = np.array([point3[0],point3[1]])

    pointend = (hom_0_end) @ np.array([0, 0, 0, 1])
    pointend_xy = np.array([pointend[0],pointend[1]])

    # ---------------------------------------------------- 

    # Remember to return values from a function
    # Method 1: The usual way we're familiar with
    # return point1_xy, point2_xy, point3_xy, pointend_xy

    # Method 2: Create one single output var (data-structure type) and then return that single output
    coords = SimpleNamespace(
        point1_xy = point1_xy, 
        point2_xy = point2_xy, 
        point3_xy = point3_xy, 
        pointend_xy = pointend_xy
        )
    return coords


# Calling the returend values from the  function
# Method 1: The usual way we're familiar with but having to manually write out each var
# point1_xy, point2_xy, point3_xy, pointend_xy = forward_kinematics(angles,params)
# print(point1_xy)
# print(point2_xy)
# print(point3_xy)
# print(pointend_xy)

# Method 2: Calling that one single output array-type, this method makes it less prone to missing out a returned value
# Sometimes you may only pass for 3 inputs but there are 4 outputs from the function and that causes error, this method resolves that
# coords = forward_kinematics(angles, params)
# print('Point1_x & y:\n', coords.point1_xy)
# print('Point2_x & y:\n', coords.point2_xy)
# print('Point3_x & y:\n', coords.point3_xy)
# print('PointEnd_x & y:\n', coords.pointend_xy)


# =============================================================================
# Animations (Construct a simple kinematic model)
# =============================================================================
# input into this func will be the returned x&y coords value from forward_kinematics()
def animate(coords):

    # reassign the name for easier callback
    point1_xy = coords.point1_xy
    point2_xy = coords.point2_xy
    point3_xy = coords.point3_xy
    pointend_xy = coords.pointend_xy

    # Construct a model of the arm using lines, it should show 3 links
    # From Base to end-tip of Link 1 - using Point 1 and Point 2
    plt.plot([point1_xy[0], point2_xy[0]], [point1_xy[1], point2_xy[1]], linewidth = 6, color = 'red')

    # From end-tip of Link 1 to end-tip of Link 2 - using Point 2 and Point 3
    plt.plot([point2_xy[0], point3_xy[0]], [point2_xy[1], point3_xy[1]], linewidth = 6, color = 'green')

    # From end-tip of Link 2 to end-tip of Link 3 - using Point 3 and Point End
    plt.plot([point3_xy[0], pointend_xy[0]], [point3_xy[1], pointend_xy[1]], linewidth = 6, color = 'blue')


    # Indicate the axes
    plt.xlabel('x')
    plt.ylabel('y')


    # Set the plot limits (recall, x limit must be more than 2.25)
    plt.xlim(-3, 3)
    plt.ylim(-3, 3)
    plt.gca().set_aspect('equal')


    # Show the plot
    # Method 1: Plot window pops up, 5 second timer before it auto-closes
    # plt.show(block=False)
    # plt.pause(5)

    # Method 2: Plot windows pops up, need user to manually close
    plt.show()

# animate(coords)
