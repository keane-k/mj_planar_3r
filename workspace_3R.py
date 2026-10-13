import numpy as np
from fk_3R import forward_kinematics, animate
from matplotlib import pyplot as plt

# =============================================================================
# This file is meant to:
# Compute the reachable workspace by 3R_planar
# We will include just a few angle combinations to map the reachable space since the combinations are infinite
# =============================================================================


# Initialise your link parameters
l1 = 1
l2 = 1
l3 = 0.25


# Set your range limits for theta1, theta2, theta3
angle_range = np.array([[0, np.deg2rad(90)],
                        [-np.deg2rad(90), np.deg2rad(90)],
                        [-np.deg2rad(90), np.deg2rad(90)]])


# initialize empty lists that contains the end-effector's x and y coords
end_x = []
end_y = []


# we'll use `np.linspace()`` to generate points across each angle's lower and upper range limit
# recall: linspace(start, stop, numbers_of_samples_to_generate); inclusive of `stop` value
# sample size of 30 will suffice
theta1 = np.linspace(angle_range[0,0], angle_range[0,1], 30)
theta2 = np.linspace(angle_range[1,0], angle_range[1,1], 30)
theta3 = np.linspace(angle_range[2,0], angle_range[2,1], 30)


# Loop through each theta value, call them using theta[index], 
# push into forward_kinematics(angles, params), grab the returned coords, 
# append pointend_xy into end_x and end_y respectively
for i1 in range(0, len(theta1)):
    for i2 in range(0, len(theta2)):
        for i3 in range(0, len(theta3)):
            
            # reassign angles and params, prep them to be inputs into forward_kinematics()
            angles = np.array([theta1[i1], theta2[i2], theta3[i3]])
            params = (l1, l2, l3)

            # call forward_kinematics() 
            coords = forward_kinematics(angles, params)

            # append end-effector's x & y coords, remember we're calling coords.pointend_xy
            end_x.append(coords.pointend_xy[0])
            end_y.append(coords.pointend_xy[1])


# Plotting Block
# Illustrate the plot, we'll put a dot at each point in space the end-effector touches, this is how we'll see the reachable workspace
plt.figure(figsize=(7,7)) # new figure window sized 7x7 inches
plt.scatter(end_x, end_y, marker = 'o', color = 'red', label = 'reachable workspace')

# Rmbr to add axis labels, title, legends (if needed)
plt.xlabel('X coords')
plt.ylabel('Y coords')
plt.title('Reachable Workspace (Sampled Approx. Grid: 30 x 30 x 30)')
plt.legend() # shows what label = 'reachable workspace' set in `scatter` refers to 

# Set the plot limits (recall, x limit must be more than 2.25)
plt.xlim(-2.5, 2.5)
plt.ylim(-2.5, 2.5)
plt.gca().set_aspect('equal') # `gca()` = get current axes
                              # `set_aspect('equal')` forces 1:1 visual ratio between x & y axis,
                              # else matplotlib default stretches the plot to fill the figure box

# Let's add a grid
plt.grid()

# Show the plot
plt.show()