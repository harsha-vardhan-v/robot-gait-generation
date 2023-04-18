#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties

switch_controller = rospy.ServiceProxy('rupert/controller_manager/switch_controller', SwitchController)
load_controller = rospy.ServiceProxy('rupert/controller_manager/load_controller', LoadController)
unload_controller = rospy.ServiceProxy('rupert/controller_manager/unload_controller', UnloadController)
get_physics_properties = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)


def call_spawn_model(model_name: str, model_xml: str, robot_namespace: str, initial_pose: Pose, reference_frame: str):
    try:
        rospy.wait_for_service('/gazebo/delete_model')
        spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        spawn_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
        rospy.loginfo(f'Spawned {model_name}')

    except rospy.ServiceException as e:
        rospy.logwarn(e)

def call_delete_model(model_name: str):
    try:
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        delete_model(model_name)
        rospy.loginfo(f'Deleted {model_name}')

    except rospy.ServiceException as e:
        rospy.logwarn(e)

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

def start_simulation():
    try:
        physics_properties = get_physics_properties()
        physics_properties.pause = False

        set_physics_properties(physics_properties.time_step, physics_properties.max_update_rate,
                        physics_properties.gravity, physics_properties.ode_config, physics_properties.pause)

    except rospy.ServiceException as e:
        rospy.logwarn(e)

def pause_simulation():
    try:
        unload_controllers()
        call_delete_model('rupert')

        physics_properties = get_physics_properties()
        physics_properties.pause = True

        set_physics_properties(physics_properties.time_step, physics_properties.max_update_rate,
                        physics_properties.gravity, physics_properties.ode_config, physics_properties.pause)

    except rospy.ServiceException as e:
        rospy.logwarn(e)