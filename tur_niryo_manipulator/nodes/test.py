import os
import sys
import time
import tty
import termios
import select
import rospy
from geometry_msgs.msg import Twist
from pyniryo import *

msg = """
Control Your TurtleBot3!
---------------------------
Switch to the mode control with the keyboard

1 : mode control position/vitesse
2 : mode control position/position
e : exit mode

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0
# niryo connection
robot_ip = "127.1.0.0"
client = NiryoRobot(robot_ip)
client.calibrate(CalibrateMode.AUTO)
client.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
client.update_tool()
client.release_with_tool()
client.set_jog_control(True)

def exit(): 
    client.set_jog_control(False)
    client.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0)
    client.set_learning_mode(True)
    client.close_connection()
    exit()

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while True:
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def control_position_vitesse():
    os.system('clear')
    print(msg)
    print("Control position/vitesse activé")
    while True:
        key = getKey()
        if key:
            print(f"Touche pressée : {key}")
            if key == 'e':
                print("Control position/vitesse désactivé")
                break  
            elif key == 'a':
                
                print("pose vers l'avant")
                client.move_joints(0.0, -0.75, 0.0, 0.0, -1.57, 0.0)
            elif key == 'b':
                print("pose vers l'arriere")
                client.move_joints(-3.0, -0.75, 0.0, 0.0, -1.57, 0.0)
            else:
                if (key == '\x03'): # CTRL+C
                    break
            print("Control position/vitesse en cours...")

def control_position_position():
    os.system('clear')
    print(msg)
    print("control position/position activé")
    while True:
        key = getKey()
        if key:
            os.system('clear')
            print(msg)
            print(f"Touche pressée : {key}")
            if key == 'e':
                print("control position/position désactivé")
                break  
            
            elif key == 'a':
                print("pose vers l'avant")
                client.move_joints(0.0, -0.75, 0.0, 0.0, -1.57, 0.0)
            elif key == 'b':
                print("pose vers l'arriere")
                client.move_joints(-3.0, -0.75, 0.0, 0.0, -1.57, 0.0)
            else:
                if (key == '\x03'): # CTRL+C
                    break
            print("control position/position en cours...")


def get_pose():
    return client.get_pose().to_list()




if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('turtlebot3/cmd_vel', Twist, queue_size=10)

    robot_pose = client.get_pose().to_list()
    robot_joints = client.get_joints()
    target_linear_vel=robot_pose[0]
    target_angular_vel=robot_pose[2]
    #########################################

    twist = Twist()

    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    pub.publish(twist)
    
    try:
        print(msg)
        while not rospy.is_shutdown():

            key = getKey()
            if key == '1':
                #clear terminal
                os.system('clear')
                print("Control position/vitesse selectionné")
                control_position_vitesse()
            elif key == '2':
                os.system('clear')
                print("control position/position selectionné")
                control_position_position()
            else:
                if (key == '\x03'): # CTRL+C
                    break
    except:
        print(e)
        

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        
    
