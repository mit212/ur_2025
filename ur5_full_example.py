import rtde_receive
import rtde_control
import pygame
import time
import robotiq_gripper
import pyrealsense2 as rs
import numpy as np
import cv2

# Initialize the RTDE interfaces
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")
gripper = robotiq_gripper.RobotiqGripper()
gripper.connect("192.168.1.102", 63352)
gripper.activate()

# Connect to the grey Logitech joystick. Note that your team will have to build your own joystick :)
pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)

# Connect to the D435i depth camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6) # Width, height, depth format (don't change), FPS. Check the D435i camera specs for allowable resolutions and FPS.
pipeline.start(config)

y_target = None # Bottle interception point. X is aligned with the treadmill, Y is perpendicular to the treadmill.
move_to_target = False # Flag to move to the bottle interception point.

while(True):

    frames = pipeline.poll_for_frames()
    if frames: # New depth frame has arrived
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        
        depth_image = depth_image[350:500,520:800] # Crop image to only the relevant segment of the treadmill   
        binary_mask = np.uint8(depth_image < 1301) * 255 # Select all pixels that are less than 1.301 meters from the camera
        
        # Remove noise: first apply a median blur to reduce salt-and-pepper noise
        binary_mask = cv2.medianBlur(binary_mask, 21)
        # Further reduce noise with morphological opening (erosion followed by dilation)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)

        # Visualize the depth image
        normalized_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX) # For the purpose of visualization, change the dynamic range of the depth image to [0, 255]
        depth_visual = cv2.convertScaleAbs(normalized_depth)
        depth_colormap = cv2.applyColorMap(depth_visual, cv2.COLORMAP_JET)
        masked_depth = cv2.bitwise_and(depth_colormap, depth_colormap, mask=binary_mask)
        cv2.imshow("Cropped Depth Camera View", masked_depth)

        # Look at a slice of the binary depth image, perpendicular to the treadmill. Find the average pixel location of all non-zero pixels in this slice.
        # Non-zero pixels correspond to objects (e.g. a bottle) whose depth is less than 1.301 meters from the camera.
        nonzero_elements = []
        for i in range(280):
            if binary_mask[75,i] != 0: # The slice is extracted from halfway down the image (pixel 75 out of 150).
                nonzero_elements.append(i)
                
        if nonzero_elements and joy.get_button(0) == 1: # Bottle is detected and joystick trigger is pressed
            avg_pixel_y = sum(nonzero_elements) / len(nonzero_elements)
            y_target = 0.7 - avg_pixel_y/280.0*0.5 # Scale pixels in the camera to a y position in the global coordinate frame
            move_to_target = True
            print(y_target)

    pygame.event.get() # Call this every loop iteration to update the joystick readings
    pose = rtde_r.getActualTCPPose()

    # Map joystick axes to robot axes, using a 2D rotation matrix of 45 degrees, since the robot axes are rotated 45 degrees w.r.t. the conveyor belt axes
    joy_x = joy.get_axis(1)/4.0*0.7071 - joy.get_axis(0)/4.0*0.7071
    joy_y = joy.get_axis(1)/4.0*0.7071 + joy.get_axis(0)/4.0*0.7071
    joy_z = (-1*joy.get_button(2) + joy.get_button(3))/5.0
    joy_grip = joy.get_axis(3)
    endpoint_omega = joy.get_axis(2)/4.0 # Gripper twist

    speed = [0, 0, 0, 0, 0, 0] # TCP speed commands

    if joy.get_button(1) == 1: # Exit the script
        break

    # Implement software limits for the robot axes to prevent collisions with the camera pole
    if pose[0] > -0.3:
        speed[0] = min(joy_x, 0)
    elif pose[0] < -0.7:
        speed[0] = max(joy_x, 0)
    else:
        speed[0] = joy_x

    if pose[1] > -0.1:
        speed[1] = min(joy_y, 0)
    elif pose[1] < -0.5:
        speed[1] = max(joy_y, 0)
    else:
        speed[1] = joy_y

    if pose[2] > 0.25: # Prevent collision with conveyor belt
        speed[2] = min(joy_z, 0)
    elif pose[2] < 0.06:
        speed[2] = max(joy_z, 0)
    else:
        speed[2] = joy_z

    speed[5] = endpoint_omega

    gripper.move((int)((joy_grip+1)*127.5), 255, 10) # Control the amount the gripper is opened or closed

    if move_to_target and y_target != None and joy.get_button(0) != 1: # Trigger has been pressed and a bottle was detected (move_to_target = True), but move only when the user releases the trigger button.
        y_target = max(0.3, min(0.7, y_target))
        target_pose = [-0.7071*y_target, -0.7071*y_target, 0.06, pose[3], pose[4], pose[5]] # x target = 0 meters
        delta_pose = [target_pose[i] - pose[i] for i in range(6)]
        print("Moving to bottle lane!")
        while abs(sum(delta_pose)) > 0.01: # While TCP pose has not yet converged to desired pose
            speed = [max(-0.25, min(0.25, target_pose[i] - pose[i])) for i in range(6)] # Task-space P control with a maximum speed threshold and gain Kp = 1
            rtde_c.speedL(speed, 2, 0)
            pose = rtde_r.getActualTCPPose()
            delta_pose = [target_pose[i] - pose[i] for i in range(6)]
            time.sleep(0.1)
        print("Ready to pick up bottle")
        move_to_target = False
        
    else:
        rtde_c.speedL(speed, 2, 0)

    #pose = [round(i,2) for i in pose]
    #print(pose)
                      
    time.sleep(0.03)

# User has pressed exit button on joystick. Stop robot and close vision pipeline.
rtde_c.speedL([0,0,0,0,0,0])
pipeline.stop()
cv2.destroyAllWindows()
