#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Float64
from gazebo_services import unload_controllers, load_controllers, delete_model

# Defaults
population_size = 64
length = 128
bias = True
phased_gait = False

# Structures and constants
pop = []

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


def main():
    load_controllers()
    unload_controllers()
    delete_model('rupert')

    first_population(population_size,length)
    rospy.loginfo(f'Population: {len(pop)} x {len(pop[0])}')
    
    pass

if __name__ == '__main__':
    rospy.init_node('gait_training')
    rospy.loginfo('Started gait_training node')

    rospy.wait_for_service('/gazebo/reset_world')
    rospy.loginfo("STARTING /gazebo/reset_world")
    rospy.wait_for_service('/gazebo/delete_model')
    rospy.loginfo("STARTING /gazebo/delete_model")
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    rospy.loginfo("STARTING /gazebo/spawn_urdf_model")
    rospy.wait_for_service('rupert/controller_manager/load_controller')
    rospy.loginfo("STARTING rupert/controller_manager/load_controller")
    rospy.wait_for_service('rupert/controller_manager/switch_controller')
    rospy.loginfo("STARTING rupert/controller_manager/switch_controller")

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