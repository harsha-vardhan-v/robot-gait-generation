#!/usr/bin/env python3

import rospy
import random
import os
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from gazebo_services import unload_controllers, load_controllers, call_spawn_model, call_delete_model

# Defaults
population_size = 64
length = 128
limit = 1.6
bias = True
phased_gait = False
static_hip = True
hip_position = 0

# Structures and constants
pop = []
final_position = 0
initial_position = 0
height_count = 1
height_average = 0
pitch_average = 0
roll_average = 0
rec_count = 0
unfit = False

def create_individual(length):
    individual_string = []
    offsets = []

    for i in range(length):
        if (bias ==True):
            individual_string.append(random.randint(-8, 4))
        else:
            individual_string.append(random.randint(-6, 6))

    if (phased_gait == True):
        for i in range(3):
            offsets.append(random.randint(0, length))
        individual = [individual_string, offsets]

    else:
        individual = individual_string
    
    return individual


def first_population(pop_size, length):
	global pop
	for i in range(pop_size):
		pop.append(create_individual(length))  

def fitness(individual_given):
    global height_count
    global height_average
    global pitch_average
    global roll_average
    global unfit
    global rec_count

    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0.2
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    knee1.publish(0)
    knee2.publish(0)
    knee3.publish(0)
    knee4.publish(0)
    ankle1.publish(0)
    ankle2.publish(0)
    ankle3.publish(0)
    ankle4.publish(0)
    hip1.publish(0)
    hip2.publish(0)
    hip3.publish(0)
    hip4.publish(0)

    # Spawn the model and load controllers
    p = os.popen('rosrun xacro xacro ' + '~/Desktop/catkin_ws/src/rupert_learns/urdf/rupert.xacro')
    xml_string = p.read()
    p.close()

    call_spawn_model(model_name='rupert',model_xml=xml_string,robot_namespace='',initial_pose=pose,reference_frame='world')
    load_controllers()

    height_count = 1
    height_average = 0
    pitch_average = 0
    roll_average = 0

    if (phased_gait == True):
        individual = individual_given[0]
        offset = individual_given[1]
    else:
        individual = individual_given

    for i in range(len(individual)):
        if (static_hip == True):
            wrap = 8
            hip_offset = 0
            hip1.publish(hip_position)
            hip2.publish(hip_position)
            hip3.publish(hip_position)
            hip4.publish(hip_position)

        else:
            hip1.publish((individual[i]/bias_div)*limit)
            hip2.publish((individual[i+1]/bias_div)*limit)
            hip3.publish((individual[i+2]/bias_div)*limit)
            hip4.publish((individual[i+3]/bias_div)*limit)
            wrap = 12
            hip_offset = 4


        if (unfit == True):
            performance = -1000
            unfit = False
            unload_controllers()
            call_delete_model('rupert')

            return performance, 0, 0
        
        if (phased_gait == True):
            if(i%2==0):
                knee1.publish((individual[i]/bias_div)*limit)
                ankle1.publish((individual[i+1]/bias_div)*limit)

                if((i+offset[0]+1)<length):
                    knee2.publish((individual[(i+offset[0])]/bias_div)*limit)
                    ankle2.publish((individual[(i+offset[0]+1)]/bias_div)*limit)
                else:
                    knee2.publish((individual[(i+offset[0]-length)]/bias_div)*limit)
                    ankle2.publish((individual[(i+offset[0]+1-length)]/bias_div)*limit)
                if((i+offset[1]+1)<length):
                    knee3.publish((individual[(i+offset[1])]/bias_div)*limit)
                    ankle3.publish((individual[(i+offset[1]+1)]/bias_div)*limit)
                else:
                    knee3.publish((individual[(i+offset[1]-length)]/bias_div)*limit)
                    ankle3.publish((individual[(i+offset[1]+1-length)]/bias_div)*limit)
                if((i+offset[2]+1)<length):
                    knee4.publish((individual[(i+offset[2])]/bias_div)*limit)
                    ankle4.publish((individual[(i+offset[2]+1)]/bias_div)*limit)
                else:
                    knee4.publish((individual[(i+offset[2]-length)]/bias_div)*limit)
                    ankle4.publish((individual[(i+offset[2]+1-length)]/bias_div)*limit)
        else:
            if(i%wrap==0):
                knee1.publish((individual[i+hip_offset]/bias_div)*limit)
                knee2.publish((individual[i+hip_offset+1]/bias_div)*limit)
                knee3.publish((individual[i+hip_offset+2]/bias_div)*limit)
                knee4.publish((individual[i+hip_offset+3]/bias_div)*limit)
                ankle1.publish(((individual[i+hip_offset+4])/bias_div)*limit)
                ankle2.publish(((individual[i+hip_offset+5])/bias_div)*limit)
                ankle3.publish(((individual[i+hip_offset+6])/bias_div)*limit)
                ankle4.publish(((individual[i+hip_offset+7])/bias_div)*limit)
        
        rospy.sleep(0.2)

    performance = final_position - initial_position + height_average/height_count - roll_average/height_count - pitch_average/height_count
    unload_controllers()
    call_delete_model('rupert')

    if (unfit == True):
        performance = -1000
        unfit = False

    return performance, final_position, height_average
   
    

def main():
    load_controllers()
    unload_controllers()
    call_delete_model('rupert')

    first_population(population_size,length)
    rospy.loginfo(f'Population: {len(pop)} x {len(pop[0])}')

    for i in range(population_size):
        score, distance_temp, height_temp = fitness(pop[i])
        rospy.loginfo(f'Score: {score}')
        break

    pass

if __name__ == '__main__':
    rospy.init_node('gait_training')
    rospy.loginfo('Started gait_training node')

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

    # switch_controller = rospy.ServiceProxy('rupert/controller_manager/switch_controller', SwitchController)
    # load_controller = rospy.ServiceProxy('rupert/controller_manager/load_controller', LoadController)
    # unload_controller = rospy.ServiceProxy('rupert/controller_manager/unload_controller', UnloadController)
    # delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    rospy.loginfo('Created service proxies')

    if (bias == True):
        bias_div = 8.0
    else:
        bias_div = 6.0

    main()