#!/usr/bin/env python3

import rospy
import os
import tf
import numpy as np
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from gazebo_services import unload_controllers, load_controllers, call_spawn_model, call_delete_model

final_position = 0
best = '1 1 1 0 1 0 1 0 0 0 0 1 1 0 1 0 0 0 0 1 0 1 1 0 1 0 0 0 1 0 0 0 0 1 1 0 1 1 1 0 0 1 0 0 1 1 1 1 0 0 0 1 0 0 1 0 0 0 0 1 0 0 0 1 0 1 1 0 0 1 0 0 1 1 0 0 0 1 1 0 0 1 0 1 0 0 0 0 1 1 1 0 1 0 1 1 1 0 0 0 1 0 0 1 1 0 1 1 1 0 0 1 0 0 1 0 1 1 1 0 0 0 1 0 0 0 0 0 1 1 1 0 1 0 0 0 1 1 0 0 1 0 0 0 1 1 0 0 0 0 0 1 0 1 1 1 0 0 1 0 0 1 1 0 1 0 1 0 0 1 1 1 1 1 0 0 0 1 1 0 0 1 1 0 0 0 1 0 0 1 0 1 1 1 0 0 0 1 1 0 0 0 0 1 0 0 1 1 1 1 1 0 1 0 0 1 1 0 1 1 0 1 0 0 1 0 0 0 1 0 1 1 1 1 1 1 1 0 1 1'
best = list(map(int, best.strip().split()))
limit = 1.6

def final_position_callback(data):
    try:
        global final_position 
        final_position = math.sqrt(((data.pose[1].position.x)*(data.pose[1].position.x))+((data.pose[1].position.y)*(data.pose[1].position.y)))
    except:
        pass


def decimal_to_binary(individual):
    dec_individual = []
    for i in range(0, 240, 10):
        res = 0
        for ele in individual[i:i+10]:
            res = (res << 1) | ele

        dec_individual.append(res)

    return dec_individual

def calc_angle(a, b, c):
    return a * np.sin(b*rospy.get_time() + c)

def main():
    unload_controllers()
    call_delete_model("rupert")

    p = os.popen('rosrun xacro xacro ' + '~/Desktop/catkin_ws/src/rupert_learns/urdf/rupert.xacro')
    xml_string = p.read()
    p.close()

    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0.2
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    call_spawn_model(model_name='rupert',model_xml=xml_string,robot_namespace='',initial_pose=pose,reference_frame='world')
    load_controllers()

    hip1.publish(0)
    hip2.publish(0)
    hip3.publish(0)
    hip4.publish(0)

    # individual = decimal_to_binary(best)
    individual = [0.78539816339, 2, 3.14159265359, 0.78539816339, 2, 0, 0.2, 0, 4.71238898038, 0.2, 0, 4.71238898038, 0.78539816339, 2, 3.14159265359, 0.78539816339, 2, 0, 1.3, 0, 4.71238898038, 1.3, 0, 4.71238898038]
    rospy.loginfo('START WALKING')

    for j in range(96):
        hip1.publish(0)
        hip2.publish(0)
        hip3.publish(0)
        hip4.publish(0)

        knee1.publish(calc_angle(individual[0], individual[1], individual[2]))
        knee2.publish(calc_angle(individual[3], individual[4], individual[5]))
        knee3.publish(calc_angle(individual[6], individual[7], individual[8]))
        knee4.publish(calc_angle(individual[9], individual[10], individual[11]))
        ankle1.publish(calc_angle(individual[12], individual[13], individual[14]))
        ankle2.publish(calc_angle(individual[15], individual[16], individual[17]))
        ankle3.publish(calc_angle(individual[18], individual[19], individual[20]))
        ankle4.publish(calc_angle(individual[21], individual[22], individual[23]))

        rospy.sleep(1.2)

    performance = final_position
    rospy.loginfo(f'Performance: {performance}')


if __name__=='__main__':
    rospy.init_node('best_coordinate_walk')
    rospy.loginfo('Started best_coordinate_walk node')

    rospy.wait_for_service('/gazebo/reset_world')
    rospy.loginfo('STARTING /gazebo/reset_world')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    rospy.wait_for_service('/gazebo/delete_model')
    rospy.loginfo('STARTING /gazebo/delete_model')
    rospy.loginfo('STARTING /gazebo/spawn_urdf_model')
    rospy.wait_for_service('rupert/controller_manager/load_controller')
    rospy.loginfo('STARTING rupert/controller_manager/load_controller')
    rospy.wait_for_service('rupert/controller_manager/switch_controller')
    rospy.loginfo('STARTING rupert/controller_manager/switch_controller')

    hip1 = rospy.Publisher('/rupert/joint5_position_controller/command', Float64, queue_size=1)
    hip2 = rospy.Publisher('/rupert/joint6_position_controller/command', Float64, queue_size=1)
    hip3 = rospy.Publisher('/rupert/joint7_position_controller/command', Float64, queue_size=1)
    hip4 = rospy.Publisher('/rupert/joint8_position_controller/command', Float64, queue_size=1)
    knee1 = rospy.Publisher('/rupert/joint1_position_controller/command', Float64, queue_size=1)
    knee2 = rospy.Publisher('/rupert/joint2_position_controller/command', Float64, queue_size=1)
    knee3 = rospy.Publisher('/rupert/joint3_position_controller/command', Float64, queue_size=1)
    knee4 = rospy.Publisher('/rupert/joint4_position_controller/command', Float64, queue_size=1)
    ankle1 = rospy.Publisher('/rupert/joint9_position_controller/command', Float64, queue_size=1)
    ankle2 = rospy.Publisher('/rupert/joint10_position_controller/command', Float64, queue_size=1)
    ankle3 = rospy.Publisher('/rupert/joint11_position_controller/command', Float64, queue_size=1)
    ankle4 = rospy.Publisher('/rupert/joint12_position_controller/command', Float64, queue_size=1)
    
    rospy.loginfo('Created publishers')

    rospy.Subscriber('/gazebo/model_states', ModelStates, final_position_callback)
    rospy.loginfo('Created subscribers')

    main()