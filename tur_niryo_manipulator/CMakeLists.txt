cmake_minimum_required(VERSION 3.0.2)
project(tur_niryo_manipulator)


find_package(catkin REQUIRED COMPONENTS 
   rospy
   geometry_msgs
)

catkin_python_setup()
catkin_package(
   CATKIN_DEPENDS rospy geometry_msgs
)

###########
## Build ##
###########


#############
## Install ##
#############


catkin_install_python(PROGRAMS 
   nodes/pose_control
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf xacro  launch 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


