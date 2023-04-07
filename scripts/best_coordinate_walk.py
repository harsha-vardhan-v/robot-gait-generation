#!/usr/bin/env python3

import rospy
import os
import tf
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from gazebo_services import unload_controllers, load_controllers, call_spawn_model, call_delete_model

final_position = 0
'''best = [#1.7253738490739368, 0.018914615764068277, 0.8733157983999309, 1.0451821249070088, -1.0583580077489714, -0.25479520447826864, 6.098198209667248, -1.0986694509886497, 
        -3.4640849137495895, 6.214153425606938, -1.7335694874587966, 2.8946671243939206, -7.335268313146696, -6.322532907212197, 0.8866449237160723, -6.793427873853263, 
        6.619189100994705, -5.351199545325551, -10.81234563933354, -0.8494205344327063, -0.6658332981416053, -1.256614840215077, -6.602335353525809, 0.5692596509550412, 
        7.061050543261404, -3.3301430132826013, 1.3336578635594245, 4.260548837171468, -4.6715561273979676, -1.116627775651875, -3.6726304860431203, -0.006821810583371668, 
        -4.654723303710468, 4.8251799983328665, -4.366320315940461, -2.305753368484194, -6.337940219503081, 1.565038130303473, -4.18852587121894, 3.718940611476149, 
        5.336218235944034, -1.9580929084588412, -4.082217570849276, 1.2880800766316731, 1.3556751108010214, -3.41200456672589, -4.333156430751628, -7.0206915981939915, 
        -2.2397101406916824, -3.4797139353188324, -3.1870431669477695, -1.4847396539887416, 3.7684873783633384, 0.47156497193444435, 3.9662234516665023, 4.248913231485676, 
        #1.8971317715085354, 1.6275654159483302, -7.747800998487886, -1.7587868627899703, -6.24010231143873, -3.8022964373035104, -8.726895953564217, -5.741403618254285, 
        #2.1089012063947834, -3.289489223911795, -6.200165111723931, 1.9918994322037646, -2.276767976858948, -4.6850343159184895, 0.8952585751877546, -3.472082472279753, 
        2.6439005104679945, -3.5942701440202485, -2.4406806859193586, 5.636808752726061, 0.16039027352789748, 0.7434427579091087, -6.346639929915324, -0.2423024889724723]
'''
best = [-4.654723303710468, 4.8251799983328665, -4.366320315940461, -2.305753368484194, -6.337940219503081, 1.565038130303473, -4.18852587121894, 3.718940611476149, 
        -3.4640849137495895, 6.214153425606938, -1.7335694874587966, 2.8946671243939206, -7.335268313146696, -6.322532907212197, 0.8866449237160723, -6.793427873853263, 
        6.619189100994705, -5.351199545325551, -10.81234563933354, -0.8494205344327063, -0.6658332981416053, -1.256614840215077, -6.602335353525809, 0.5692596509550412, 
        7.061050543261404, -3.3301430132826013, 1.3336578635594245, 4.260548837171468, -4.6715561273979676, -1.116627775651875, -3.6726304860431203, -0.006821810583371668, 
        -4.654723303710468, 4.8251799983328665, -4.366320315940461, -2.305753368484194, -6.337940219503081, 1.565038130303473, -4.18852587121894, 3.718940611476149, 
        5.336218235944034, -1.9580929084588412, -4.082217570849276, 1.2880800766316731, 1.3556751108010214, -3.41200456672589, -4.333156430751628, -7.0206915981939915, 
        -2.2397101406916824, -3.4797139353188324, -3.1870431669477695, -1.4847396539887416, 3.7684873783633384, 0.47156497193444435, 3.9662234516665023, 4.248913231485676, 
        -4.654723303710468, 4.8251799983328665, -4.366320315940461, -2.305753368484194, -6.337940219503081, 1.565038130303473, -4.18852587121894, 3.718940611476149, 
        -3.4640849137495895, 6.214153425606938, -1.7335694874587966, 2.8946671243939206, -7.335268313146696, -6.322532907212197, 0.8866449237160723, -6.793427873853263, 
        6.619189100994705, -5.351199545325551, -10.81234563933354, -0.8494205344327063, -0.6658332981416053, -1.256614840215077, -6.602335353525809, 0.5692596509550412, 
        7.061050543261404, -3.3301430132826013, 1.3336578635594245, 4.260548837171468, -4.6715561273979676, -1.116627775651875, -3.6726304860431203, -0.006821810583371668, 
        -4.654723303710468, 4.8251799983328665, -4.366320315940461, -2.305753368484194, -6.337940219503081, 1.565038130303473, -4.18852587121894, 3.718940611476149, 
        5.336218235944034, -1.9580929084588412, -4.082217570849276, 1.2880800766316731, 1.3556751108010214, -3.41200456672589, -4.333156430751628, -7.0206915981939915, 
        -2.2397101406916824, -3.4797139353188324, -3.1870431669477695, -1.4847396539887416, 3.7684873783633384, 0.47156497193444435, 3.9662234516665023, 4.248913231485676, 
        -4.654723303710468, 4.8251799983328665, -4.366320315940461, -2.305753368484194, -6.337940219503081, 1.565038130303473, -4.18852587121894, 3.718940611476149, 
        -3.4640849137495895, 6.214153425606938, -1.7335694874587966, 2.8946671243939206, -7.335268313146696, -6.322532907212197, 0.8866449237160723, -6.793427873853263, 
        
        ]

#best = [-3, -3, 4, 5, -4, 3, -3, 1, -5, 4, 2, 0, -4, 1, -5, 6, 4, -4, -2, -3, 5, -4, -3, -6, -3, 6, -1, -2, -3, 3, 1, 3, -3, -3, -6, 5, 3, 3, -1, -1, 5, 0, -3, -6, 1, 2, -5, -5, -3, 2, 4, -2, 1, -2, 4, 2, -2, 6, -2, 5, -6, -6, 1, -5, 5, -5, -6, -1, -2, -1, -6, -2, 5, 2, 5, 3, -6, 1, -1, 4, -4, 3, -4, -1, -6, 1, -2, 4, 6, -4, 0, -3, 4, -2, -2, -5, -4, -2, -1, -4, 6, 2, -1, 0, 0, 1, 0, -1, -6, -1, -3, -6, 3, 1, -2, 2, -4, -6, 4, 0, 2, -4, -4, 6, -1, 3, -6, -1]
limit = 1.6

def final_position_callback(data):
    try:
        global final_position 
        final_position = math.sqrt(((data.pose[1].position.x)*(data.pose[1].position.x))+((data.pose[1].position.y)*(data.pose[1].position.y)))
    except:
        pass

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

    rospy.loginfo('START WALKING')

    for j in range(3):
        for i in range(0, len(best), 8):
            if(i%8==0):
                hip1.publish(0)
                hip2.publish(0)
                hip3.publish(0)
                hip4.publish(0)

                knee1.publish((best[i]/8.0)*limit)
                knee2.publish((best[i+1]/8.0)*limit)
                knee3.publish((best[i+2]/8.0)*limit)
                knee4.publish((best[i+3]/8.0)*limit)
                ankle1.publish((best[i+4]/8.0)*limit)
                ankle2.publish((best[i+5]/8.0)*limit)
                ankle3.publish((best[i+6]/8.0)*limit)
                ankle4.publish((best[i+7]/8.0)*limit)

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