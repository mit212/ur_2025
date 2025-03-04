from rtde_control import RTDEControlInterface as RTDEControl

rtde_c = RTDEControl("169.254.9.43")
speed = [0, 0, -0.100, 0, 0, 0]

joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]

# Move to initial joint position with a regular moveJ
rtde_c.moveJ(joint_q)

rtde_c.moveUntilContact(speed)
print(rtde_c.getJointTorques())
rtde_c.stopScript()


