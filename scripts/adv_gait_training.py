#!/usr/bin/env python3
import math
import random
from itertools import repeat

import rospy
import random
import os
import tf
import math
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from gazebo_services import unload_controllers, load_controllers, call_spawn_model, call_delete_model
from gait_training import model_state_callback

try:
    from collections.abc import Sequence
except ImportError:
    from collections import Sequence

# Defaults
population_size = 64
length = 128
limit = 1.6
bias = True
probabilistic_cull = True
generations = 150
phased_gait = False
static_hip = True
hip_position = 0
switch_mutation_gen = 100
# result_path = os.path.expanduser('~/Desktop/catkin_ws/src/rupert_learns/results')
result_path = ''

# Structures and constants
pop = []
sco = []
dis = []
hei = []
surv = []
final_position = 0
initial_position = 0
height_count = 1
height_average = 0
pitch_average = 0
roll_average = 0
ride_height = 0
rec_count = 0
unfit_count = []
unfit = False
num_unfit = 0
negative_step_count = 0
prev_pos = 0
best = [-3, -3, 4, 5, -4, 3, -3, 1, -5, 4, 2, 0, -4, 1, -5, 6, 4, -4, -2, -3, 5, -4, -3, -6, -3, 6, -1, -2, -3, 3, 1, 3, -3, -3, -6, 5, 3, 3, -1, -1, 5, 0, -3, -6, 1, 2, -5, -5, -3, 2, 4, -2, 1, -2, 4, 2, -2, 6, -2, 5, -6, -6, 1, -5, 5, -5, -6, -1, -2, -1, -6, -2, 5, 2, 5, 3, -6, 1, -1, 4, -4, 3, -4, -1, -6, 1, -2, 4, 6, -4, 0, -3, 4, -2, -2, -5, -4, -2, -1, -4, 6, 2, -1, 0, 0, 1, 0, -1, -6, -1, -3, -6, 3, 1, -2, 2, -4, -6, 4, 0, 2, -4, -4, 6, -1, 3, -6, -1]


def mutGaussian(individual, mu, sigma, indpb):
    """This function applies a gaussian mutation of mean *mu* and standard
    deviation *sigma* on the input individual. This mutation expects a
    :term:`sequence` individual composed of real valued attributes.
    The *indpb* argument is the probability of each attribute to be mutated.
    :param individual: Individual to be mutated.
    :param mu: Mean or :term:`python:sequence` of means for the
               gaussian addition mutation.
    :param sigma: Standard deviation or :term:`python:sequence` of
                  standard deviations for the gaussian addition mutation.
    :param indpb: Independent probability for each attribute to be mutated.
    :returns: A tuple of one individual.
    This function uses the :func:`~random.random` and :func:`~random.gauss`
    functions from the python base :mod:`random` module.
    """
    size = len(individual)
    if not isinstance(mu, Sequence):
        mu = repeat(mu, size)
    elif len(mu) < size:
        raise IndexError("mu must be at least the size of individual: %d < %d" % (len(mu), size))
    if not isinstance(sigma, Sequence):
        sigma = repeat(sigma, size)
    elif len(sigma) < size:
        raise IndexError("sigma must be at least the size of individual: %d < %d" % (len(sigma), size))

    for i, m, s in zip(range(size), mu, sigma):
        if random.random() < indpb:
            individual[i] += random.gauss(m, s)

    return individual

def create_individual():
    return mutGaussian(best, best, best, 0.4)

def first_population(pop_size):
	global pop
	for i in range(pop_size):
		pop.append(create_individual())
                
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

def model_state_callback(data):
    global height_count
    global final_position
    global ride_height
    global height_average
    global pitch_average
    global roll_average
    global unfit
    global negative_step_count

    try:
        quaternion = (data.pose[1].orientation.x,data.pose[1].orientation.y,data.pose[1].orientation.z,data.pose[1].orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = math.sqrt(euler[0]*euler[0])
        pitch = math.sqrt(euler[1]*euler[1])
        yaw = math.sqrt(euler[2]*euler[2])

        if roll>0.785398 or pitch>0.785398:
            rospy.loginfo('Flipped')
            unfit = True

        if (data.pose[1].position.x - prev_pos < 0):
            negative_step_count += 1

        prev_pos = data.pose[1].position.x

        if negative_step_count > 2:
            negative_step_count = 0
            unfit = True
        
        height_count += 1
        ride_height = data.pose[1].position.z
        pitch_average += pitch
        roll_average += roll
        height_average += ride_height
        final_position = math.sqrt(((data.pose[1].position.x)*(data.pose[1].position.x)))


    except:
        pass

def main():
    global pop
    global sco
    global dis
    global hei
    global num_unfit
    global unfit_count

    pop = []
    sco = []
    dis = []
    hei = []

    running_fitness = []
    running_height = []
    running_dist = []

    generation = 0

    load_controllers()
    unload_controllers()
    call_delete_model('rupert')

    first_population(population_size)
    rospy.loginfo(f'Population: {len(pop)} x {len(pop[0])}')

    for i in range(population_size):
        score, distance_temp, height_temp = fitness(pop[i])
        
        while score == -1000:
            num_unfit += 1
            pop[i] = create_individual(length)
            score, distance_temp, height_temp = fitness(pop[i])
            rospy.loginfo('Replacing pop 0 individual')

        rospy.loginfo(f'Individual {i}: {score}')
        sco.append(score)
        dis.append(distance_temp)
        hei.append(height_temp)

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

    rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)
    rospy.loginfo('Created subscribers')

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