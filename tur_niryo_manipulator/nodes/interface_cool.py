import rospy
import tkinter as tk
from geometry_msgs.msg import Twist
from pyniryo import *

# Replace this with the IP address of your Niryo robot
robot_ip = "127.1.0.0"
robot = NiryoRobot(robot_ip)
robot.calibrate(CalibrateMode.AUTO)
robot.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
robot.update_tool()
robot.release_with_tool()
robot.set_jog_control(True)

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1
speed_scaling_factor = 1.2 # Adjust this value based on your requirements

target_linear_vel = 0.0
target_angular_vel = 0.0
control_linear_vel = 0.0
control_angular_vel = 0.0

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

def control_turtlebot(control_mode, pub):
    global target_linear_vel, target_angular_vel, control_linear_vel, control_angular_vel

    robot_pose = robot.get_pose().to_list()
    robot_joints = robot.get_joints()
    # print(f"Robot Pose: {robot_pose}")
    target_linear_vel = robot_pose[0]
    target_angular_vel = robot_joints[0]  # Modify this line based on your specific requirement

    twist = Twist()

    if control_mode == "position/position":
        # Position/Position mode
        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
    elif control_mode == "position/vitesse":
        # Position/Vitesse mode with speed scaling
        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        control_linear_vel = control_linear_vel * speed_scaling_factor
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    pub.publish(twist)
    # print(f"Control: linear_vel={twist.linear.x}, angular_vel={twist.angular.z}")

    # Schedule the next call to control_turtlebot after 100 milliseconds
    root.after(100, control_turtlebot, control_mode, pub)

def move_to_default_pose():
    robot.move_joints([-2.96, 0, 0, 0, 0, 0])

def move_to_pose_1():
    robot.move_joints([0.0, 0.0, 0.0, 0.0, -1.57, 0.0])

def move_to_pose_2():
    robot.move_joints([0, -0.251, 0.282, 0, -1.57, 0])

def start_control(pub):
    selected_mode = mode_var.get()
    print(f"Start Control: Selected Mode - {selected_mode}")
    # Start the control_turtlebot function
    control_turtlebot(selected_mode, pub)

def stop_control(pub):
    global target_linear_vel, target_angular_vel, control_linear_vel, control_angular_vel
    twist = Twist()
    # Stop the control by publishing a Twist message with zero velocities
    twist.linear.x = 0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0

    pub.publish(twist)
    print("Stop Control: Control Stopped")

def on_closing(root, pub):
    # Function to handle window closure events
    stop_control(pub)
    root.destroy()
    print("Window Closed")

# Create the main window
root = tk.Tk()
root.title("Niryo Robot Control")

# Create a variable to store the selected control mode
mode_var = tk.StringVar(value="position/position")

# Create radio buttons for control mode selection
position_position_radio = tk.Radiobutton(root, text="Position/Position", variable=mode_var, value="position/position")
position_vitesse_radio = tk.Radiobutton(root, text="Position/Vitesse", variable=mode_var, value="position/vitesse")

# Create buttons for control actions
start_button = tk.Button(root, text="Start Control", command=lambda: start_control(pub))
stop_button = tk.Button(root, text="Stop Control", command=lambda: stop_control(pub))
quit_button = tk.Button(root, text="Quit", command=lambda: on_closing(root, pub))

# Create buttons for moving the robot to predefined poses
default_pose_button = tk.Button(root, text="Move to Default Pose", command=move_to_default_pose)
pose_1_button = tk.Button(root, text="Move to Pose 1", command=move_to_pose_1)
pose_2_button = tk.Button(root, text="Move to Pose 2", command=move_to_pose_2)

# Pack the widgets
position_position_radio.pack()
position_vitesse_radio.pack()
start_button.pack()
stop_button.pack()
quit_button.pack()
default_pose_button.pack()
pose_1_button.pack()
pose_2_button.pack()

# Setup the ROS node and publisher
rospy.init_node('turtlebot3_teleop')
pub = rospy.Publisher('turtlebot3/cmd_vel', Twist, queue_size=10)

# Configure the window closure event
root.protocol("WM_DELETE_WINDOW", lambda: on_closing(root, pub))

# Start the GUI event loop
root.mainloop()
