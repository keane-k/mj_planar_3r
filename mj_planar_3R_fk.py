# =============================================================================
# Forward Kinematics & MuJoCo in one file
# - generate poses via Forward Kinematics and simulate using MuJoCo's physics engine
# - handle graphical windows, camera config, etc. (refer to MuJoCo docs)
# =============================================================================

import mujoco as mj
from mujoco.glfw import glfw # GLFW is a separate, general-purpose library (not MuJoCo specific) for creating OS windows,
                             # handling keyboard/mouse input, managing OpenGL rendering context.
                             # FYI: MuJoCo ships a bundled copy of it (mujoco.glfw) so you don't need to install GLFW separately. 
import numpy as np 
import os # path handling module
from fk_3R import forward_kinematics # call forward_kinematics() from fk_3R.py


# =============================================================================
# Path Setup
# - Get file path to XML file
# =============================================================================
# work out where this script lives
dirname = os.path.dirname(os.path.abspath(__file__)) # __file__ is the path of this script, we will use abspath() to turn it into a full path
                                                     # (ie. /home/user/MuJoCo/launch_mujoco.py)
                                                     # dirname() strips the file name, leaving only the folder (ie. /home/user/MuJoCo)
                                                     # this allows the script to work no matter which folder we run it from

# build path to the input file we want
xml_path = os.path.join(dirname, "planar_3R.xml") # os.path.join(folder, name) glues them together with the correct separator for our OS


# =============================================================================
# Inputs for Simulation
# - Simulation time (simulation ends on this timer)
# - Toggle (On/Off) printing camera config (to get azimuth, elevation, distance, lookat point)
# =============================================================================
set_timer = 0 # 0 - No, 1 - Yes
sim_end = 100 # timer in secs
print_camera_config = 0 # 0 - No, 1 - Yes
                        # prints current camera config per frame
                        # 1) Manually adjust the view in simulation according to your liking
                        # 2) Grab current camera config, make it the default setting via commands


# =============================================================================
# Setup & initialize Controllers functions (keyboard, mouse)
# =============================================================================
# Globals for mouse state (for callback functions)
button_left = False
button_middle = False
button_right = False
lastx = 0 # last known x pos
lasty = 0 # last known y pos


def init_controller(model,data):
    # initialize controller (placeholder), this func is called once, at the start (ie. set initial joint positions)
    pass


def controller(model,data):
    # put controller here, this func will be called inside the simulation
    pass


# keyboard callback 
# func 'records' keyboard presses; 
# GLFW calls this func automatically every time a key is PRESSED or RELEASED, it will pass 5 info:
# 1) which `window` it happened it;
# 2) which `key`;
# 3) hardware-specific `scancode` (rarely used directly);
# 4) the `action` (press / release / repeat);
# 5) any `modifier` keys (shift, ctrl);
def keyboard(window, key, scancode, action, modifier):
    # this func will just check for one specific case: <backspace> being pressed
    if action == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        # -> <mj.mj_resetData(m,d)> zeroes out the simultation state
        mj.mj_resetData(model, data)

        # -> <mj.mj_forward(m,d)> recomputes all derived quantities (body positions, etc.) from that reset state 
        # for the scene to be redrawn correctly at the reset pose instead of a stale one
        mj.mj_forward(model, data)


# mouse button callback
# func 'records' mouse button presses; passes the same 5 info from <keyboard()>
# GLFW calls this func whenever any mouse button changes state (PRESS or RELEASE)
def mouse_button(window, button, action, modifier):
    # Update global mouse states
    global button_left
    global button_middle
    global button_right

    # Update button states
    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # Update mouse position (cursor position) in the pop-up 'window'
    glfw.get_cursor_pos(window)


# mouse move callback
# func 'tracks' continuously as the mouse moves across the window
# 2D screen - only x & y-axis
def mouse_move(window, xpos, ypos):
    # compute mouse displacement and update them
    global button_left
    global button_middle
    global button_right 
    global lastx # last known x pos
    global lasty # last known y pos

    # compute how far the mouse moved since the last cell
    # Note: we're finding displacement, not the absolute position --> rotate/pan/zoom of camera done incrementally, not absolute
    dx = xpos - lastx
    dy = ypos - lasty

    # and then update last known x and y to the current pos
    lastx = xpos
    lasty = ypos

    # if no mouse buttons currently held down, no action to be taken (referencing global states from mouse_button())
    # Translate: moving the mouse w/o clicking shouldn't move the camera
    if (not button_left) and (not button_middle) and (not button_right):
        return # no action to be taken, then function exits early via `return`

    # get current window size, follow this format <width, height = glfw.get_window_size(specify_which_window)>
    width, height = glfw.get_window_size(window)

    # check if modifier is applied (ie. shift key - we're interested in cam nav only), capture the key press
    # - LEFT SHIFT & RIGHT SHIFT are 2 separate keys but functions the same 
    PRESS_LEFT_SHIFT = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    # register that left_shift or right_shift are the same modifier
    shift_modifier = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)
    
    # define the different condition states of mouse button w/ or w/o shift_modifier
    # right mouse button --> pans the camera
    if button_right:
        if shift_modifier:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V

    # left mouse button --> rotates the camera
    elif button_left:
        if shift_modifier:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V

    # middle mouse button (scroll wheel) --> zooms camera in and out
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    # move the camera based on 'action'
    # dx/height & dy/height for snappier controls, dx/width & dy/height will make x-movement feels sluggish compared to y-movement
    mj.mjv_moveCamera(model, action, dx/height, dy/height, cam)


# scroll wheel callback
# func 'records' scroll wheel movements
def scroll_wheel(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    
    # emulate vertical mouse motion = 5% of window height (y-axis)
    # (+ve y): scroll up = zoom in, scroll down = zoom out
    # (-ve y): scroll up = zoom out, scroll down = in
    mj.mjv_moveCamera(model, action, 0.0, 0.05 * yoffset, cam)


# =============================================================================
# Creating the model, data and visualization structures
# - Load the model and create simulation data from an XML file
# - Load the simulation data -- contains the physics simulation state data 
# - Load abstract camera -- [TO BE INITIALIZED LATER]
# - Load scene visualization options -- [TO BE INITIALIZED LATER]
# =============================================================================
# MjModel is the model: bodies, joints, geoms, etc. [Compiled from XML file]
# Note: It will not change while the simulation is running.
model = mj.MjModel.from_xml_path(xml_path)

# MjData is the simulation state: positions, velocities, forces
# These states changes at every physics step and is built from the model
data = mj.MjData(model)

# MjvCamera handles how the 3D scene is viewed: target, angles, zoom and tracking --> azimuth, elevation, distance, look-at point
# It controls the observer's viewpoint, NOT the physical camera sensors in the model (if any)
cam = mj.MjvCamera() # abstract camera

# MjvOption configures what visual elements are drawn or hidden in the 3D scene
# Toggling flags for rendering bones, contacts, joints, frames, shadows, etc.
visual_opt = mj.MjvOption() # visualization options


# =============================================================================
# Initialize GLFW window setup
# - initialize GLFW library (must be called before any other GLFW functions)
# - install GLFW mouse and keyboard callbacks
# =============================================================================
glfw.init() # initializing GLFW library, rmbr to call this before other GLFW funcs
window = glfw.create_window(1200, 900, "Demo_Test", None, None) # opens an OS window, 1200x900 pixels, titled "Demo_Test"
                                                                # Two optional `None` arguments are for:
                                                                # 1) monitor (fullscreen)
                                                                # 2) window-sharing parameters
glfw.make_context_current(window) # tells graphics driver that subsequent OpenGL (Open Graphics Library) drawing commands 
                                  # should target the created 'window'
                                  # Note: this step is necessary bcuz an application can have multiple windows open.
glfw.swap_interval(1) # enable v-sync, caps rendering to monitor's refresh rate (Hz), prevents visual tearing

# mouse & keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll_wheel)


# =============================================================================
# Initialize visualization data structures
# - Initialize camera
# - Initialize visulization options
# - Create scene & context
# =============================================================================
mj.mjv_defaultCamera(cam)           # callback to MjvCamera()
mj.mjv_defaultOption(visual_opt)    # callback to MjvOption()

scene = mj.MjvScene(model, maxgeom = 10000) # allocate a buffer to hold the actual 3D geometry to be drawn each frame
                                            # (positions, orientations, colors of every visible shape)
                                            # maxgeom = 10000 --> reserves space for up to 10,000 visual elements
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value) # sets up the actual OpenGL rendering context:
                                                                       # shaders, fonts, buffers need to draw `scene` onto window
                                                                       # FONTSCALE sets UI text to 150% scale (optional)    


########
# =============================================================================
# Set up Camera Configuration, optional but QOL improvement from having to always manually adjust camera in MuJoCo
# - azimuth
# - elevation
# - distance
# - lookat
# =============================================================================
# recall, we've addressed `cam = mj.MjvCamera()`
cam.azimuth = 90.0
cam.elevation = -89.0
cam.distance = 7.178388686350998
cam.lookat = np.array([ 0.0 , 0.0 , 0.0 ])

#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)


# =============================================================================
# Run Main Loop (REF MJ's template)
# =============================================================================

# setting up keyframe (1) get id of the pos first, (2) callback id to get the pos using qpos
key_id = model.key('home_pos').id # get keyframe id (refer back to xml file)
key_qpos = model.key_qpos[key_id] # get value inside the id (using qpos)
print(key_qpos)

# <while> loop runs once per drawn frame and stops when user closes the window
while not glfw.window_should_close(window):
    # Simulation time is <data.time>, MuJoCo's internal clock, it only moves when you tell it to
    # Real time is your screen drawing about 60 frames per second 
    # In C, this is reflected as <mjtNum simstart = d->time;>
    time_prev = data.time # stores a snapshot of the simulation clock

    # main feature, all your math equations will fall under this section
    while(data.time - time_prev < 1.0/60.0): # <data.time - time_prev> is how much simulated time has passed during this frame
                                             # 1.0/60 = 0.0167s; the length of one frame at 60Hz
                                             # tl;dr keep doing the work below until one frame's worth of simulated time has passed
        
        # integrates the equation (Dynamic simulation; computes acceleration, integrate to get velocity and then position)
        # mj.mj_step(model, data)

        # manual time advancement (needed with forward, else MuJoCo crashes)
        data.time += 0.2 # Note: time has no significance here, we just need it for animations
        # print(data.time)

        # HOW TO CALL ANGLES USING MUJOCO <qpos>
        # ---------------------------------------
        # (1) storing angles directly to qpos
        # data.qpos = np.array([1.5707, -1.5707, -1.5707]) # home pos, as defined in <planar_3R.xml>

        # (2) calling angles from keyframes
        data.qpos = key_qpos.copy()
        # ---------------------------------------

        # <mujoco.mj_forward(m,d)> computes current state: positions, forces, accelerations but does not MOVE TIME or INTEGRATE MOTION
        mj.mj_forward(model, data) # we just want forward kinematics, we don't need time to move forward 
                                   # but MJ will require time to move forward else it'll lead to crash
                                   # TO RESOLVE: we manually advance time (as shown above)
        
        # compute end_effector's pos
        mj_ee_pos = data.site('end-effector').xpos
        print('mj_ee_pos')
        print(mj_ee_pos)


        # call forward_kinematics(angles, params) 
        # angles can be taken keyframe <qpos>
        angles = data.qpos.copy()
        # set up params
        l1 = 1
        l2 = 1
        l3 = 0.25
        params = (l1, l2, l3) 

        coords = forward_kinematics(angles, params)

        # check python_ee result to mujoco_ee result
        py_ee_pos = coords.pointend_xy
        print('\npy_ee_pos')
        print(py_ee_pos,'\n')
        
    # if user specified a timer earlier, when timer reached, terminate everything
    if(set_timer == 1):
        if(data.time >= sim_end):
            break


    # get framebuffer viewpoint
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)


    # If user toggles,
    # print camera configuration into terminal
    # User can use this configs to set a default camera view angle instead of always having to manually adjust
    if(print_camera_config == 1):
        print('cam.azimuth = ', cam.azimuth)
        print('cam.elevation = ', cam.elevation)
        print('cam.distance = ', cam.distance)
        print('cam.lookat = np.array[(', cam.lookat[0], ',', cam.lookat[1], ',', cam.lookat[2], ')]')


    # update scene and render
    mj.mjv_updateScene(model, data, visual_opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events() 


# terminate and wipe everything
glfw.terminate()

print("key_qpos :", key_qpos)
print("data.qpos:", data.qpos)
print("angles   :", angles)