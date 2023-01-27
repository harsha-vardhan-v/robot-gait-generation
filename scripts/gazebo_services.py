#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController
from gazebo_msgs.srv import DeleteModel, SpawnModel

switch_controller = rospy.ServiceProxy('rupert/controller_manager/switch_controller', SwitchController)
load_controller = rospy.ServiceProxy('rupert/controller_manager/load_controller', LoadController)
unload_controller = rospy.ServiceProxy('rupert/controller_manager/unload_controller', UnloadController)
spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

def load_controllers():
    try:
        rospy.wait_for_service('rupert/controller_manager/load_controller')
        rospy.wait_for_service('rupert/controller_manager/switch_controller')

        controllers = ['joint_state_controller', '/rupert/joint1_position_controller', '/rupert/joint2_position_controller','/rupert/joint3_position_controller','/rupert/joint4_position_controller','/rupert/joint5_position_controller','/rupert/joint6_position_controller','/rupert/joint7_position_controller','/rupert/joint8_position_controller','/rupert/joint9_position_controller','/rupert/joint10_position_controller','/rupert/joint11_position_controller','/rupert/joint12_position_controller']

        for controller in controllers:
            load_controller(controller)

        switch_controller(start_controllers = controllers, stop_controllers = [], strictness = 2)
        rospy.loginfo('Loaded controllers')
        return True

    except rospy.ServiceException as e:
        rospy.logwarn(e)

def unload_controllers():
    try:
        rospy.wait_for_service('rupert/controller_manager/load_controller')
        rospy.wait_for_service('rupert/controller_manager/switch_controller')

        controllers = ['joint_state_controller', '/rupert/joint1_position_controller', '/rupert/joint2_position_controller','/rupert/joint3_position_controller','/rupert/joint4_position_controller','/rupert/joint5_position_controller','/rupert/joint6_position_controller','/rupert/joint7_position_controller','/rupert/joint8_position_controller','/rupert/joint9_position_controller','/rupert/joint10_position_controller','/rupert/joint11_position_controller','/rupert/joint12_position_controller']

        switch_controller(start_controllers = [], stop_controllers = controllers, strictness = 2)
        for controller in controllers:
            unload_controller(controller)

        rospy.loginfo('Unloaded controllers')

    except rospy.ServiceException as e:
        rospy.logwarn(e)