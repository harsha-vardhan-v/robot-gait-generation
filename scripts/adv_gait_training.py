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
population_size = 4
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
final_disp = 0
prev_pos = 0

#best = [-1.802456988075699, -4.554392176546572, 3.035264569765163, 6.920622613954937, -6.258116001184408, 4.328464860351994, -2.583477466465817, 0.9092450788700558, -2.8276544664127377, 4.717370712219781, 2.4422394390635644, -1.3321222625832272, -5.710443982921806, 3.412983367619702, -7.015772532157516, 6.989589964856525, 1.910614177992259, -3.180628273920048, -1.2117570864012124, -2.20444052382357, 4.282537237536275, -4.266395196852301, -1.5998455692619278, -3.540818143150492, -3.658438858801776, 1.484623808008094, -0.976331384493019, -3.1670499702957664, -2.811243781667011, 0.6736349575426153, 2.924750641581015, 4.940622161352684, -2.380483415151488, -5.834886666538785, -5.343705010618698, 5.195406286420664, 4.406607762628701, 1.0204764971682887, -1.1282562964670544, 1.7732663535257882, 6.79157755763823, -1.1408355798387277, -3.375136110042621, -5.798721113522597, 1.970129214037349, -0.45890766510017056, -2.4022407307104334, -7.378683543517729, 1.7253738490739368, 0.018914615764068277, 0.8733157983999309, 1.0451821249070088, -1.0583580077489714, -0.25479520447826864, 6.098198209667248, -1.0986694509886497, -3.4640849137495895, 6.214153425606938, -1.7335694874587966, 2.8946671243939206, -7.335268313146696, -6.322532907212197, 0.8866449237160723, -6.793427873853263, 6.619189100994705, -5.351199545325551, -10.81234563933354, -0.8494205344327063, -0.6658332981416053, -1.256614840215077, -6.602335353525809, 0.5692596509550412, 7.061050543261404, -3.3301430132826013, 1.3336578635594245, 4.260548837171468, -4.6715561273979676, -1.116627775651875, -3.6726304860431203, -0.006821810583371668, -4.654723303710468, 4.8251799983328665, -4.366320315940461, -2.305753368484194, -6.337940219503081, 1.565038130303473, -4.18852587121894, 3.718940611476149, 5.336218235944034, -1.9580929084588412, -4.082217570849276, 1.2880800766316731, 1.3556751108010214, -3.41200456672589, -4.333156430751628, -7.0206915981939915, -2.2397101406916824, -3.4797139353188324, -3.1870431669477695, -1.4847396539887416, 3.7684873783633384, 0.47156497193444435, 3.9662234516665023, 4.248913231485676, 1.8971317715085354, 1.6275654159483302, -7.747800998487886, -1.7587868627899703, -6.24010231143873, -3.8022964373035104, -8.726895953564217, -5.741403618254285, 2.1089012063947834, -3.289489223911795, -6.200165111723931, 1.9918994322037646, -2.276767976858948, -4.6850343159184895, 0.8952585751877546, -3.472082472279753, 2.6439005104679945, -3.5942701440202485, -2.4406806859193586, 5.636808752726061, 0.16039027352789748, 0.7434427579091087, -6.346639929915324, -0.2423024889724723]

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

def create_gaussian_individual(individual):
    return mutGaussian(individual, 0, 1, 0.8)

def first_population(pop_size):
    global best
    global pop

    for i in range(pop_size):
        pop.append(create_gaussian_individual(best))
                
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

    initial_time = rospy.get_rostime()
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

    final_time = rospy.get_rostime()

    performance = (final_position - initial_position) / (final_time.secs - initial_time.secs) + height_average/height_count - roll_average/height_count - pitch_average/height_count
    unload_controllers()
    call_delete_model('rupert')

    if (unfit == True or final_disp < 0):
        performance = -1000
        unfit = False

    return performance, final_position, height_average

def best_two(break_point, scores):
    index1 = np.argmax(scores[:break_point])
    index2 = np.argmax(scores[break_point:]) + break_point

    rospy.loginfo(f'Breakpoint: {break_point}, index1: {index1}, index2: {index2}')

    return ([pop[index1], pop[index2]], [scores[index1], scores[index2]])

def two_point_crossover(individual1, individual2):
    child1 = individual1
    child2 = individual2

    for i in range(2):
        index = random.randint(0, len(individual1)-1)
        child1 = child1[:index]+child2[index:]
        child2 = child2[:index]+child1[index:]

    return child1, child2

def model_state_callback(data):
    global height_count
    global final_position
    global ride_height
    global height_average
    global pitch_average
    global roll_average
    global unfit
    global final_disp

    try:
        quaternion = (data.pose[1].orientation.x,data.pose[1].orientation.y,data.pose[1].orientation.z,data.pose[1].orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = math.sqrt(euler[0]*euler[0])
        pitch = math.sqrt(euler[1]*euler[1])
        yaw = math.sqrt(euler[2]*euler[2])

        if roll>0.785398 or pitch>0.785398:
            rospy.loginfo('Flipped')
            unfit = True
        
        height_count += 1
        ride_height = data.pose[1].position.z
        pitch_average += pitch
        roll_average += roll
        height_average += ride_height
        final_position = math.sqrt(((data.pose[1].position.x)*(data.pose[1].position.x)))
        final_disp = data.pose[1].position.x


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
            pop[i] = create_gaussian_individual(best)
            score, distance_temp, height_temp = fitness(pop[i])
            rospy.loginfo('Replacing pop 0 individual')

        rospy.loginfo(f'Individual {i}: {score}')
        sco.append(score)
        dis.append(distance_temp)
        hei.append(height_temp)

    rospy.loginfo(f'Sco: {sco}')
    rospy.loginfo(f'Dis: {dis}')
    rospy.loginfo(f'Hei: {hei}')

    fit = np.array(sco)
    height_arr = np.array(hei)
    dist_arr = np.array(dis)

    unfit_count.append(num_unfit)
    num_unfit = 0

    while(generation<generations):
        generation += 1
        rospy.loginfo(f'Generation: {generation}')

        [individual1, individual2], [p_score1, p_score2] = best_two(population_size//2, fit)
        rospy.loginfo(f'Parent scores: {p_score1}, {p_score2}')

        pop = []
        sco = []
        dis = []
        hei = []

        i=0
        while i < population_size:
            choice = random.choice([0,1])

            child1 = []
            child2 = []
            # Guassian mutation of a parent
            if choice == 0:
                rospy.loginfo('Guassian mutation of parent')
                child1 = mutGaussian(individual1, 0, 1, 0.4)
                child2 = mutGaussian(individual2, 0, 1, 0.4)
            
            # Two point crossover
            else:
                rospy.loginfo('Two point crossover')
                child1, child2 = two_point_crossover(individual1, individual2)

            fitness_temp, distance_temp, height_temp = fitness(child1)
            while (fitness_temp == -1000):
                num_unfit = num_unfit + 1
                child1 = mutGaussian(best, 0, 1, 0.7)
                fitness_temp, distance_temp, height_temp = fitness(child1)
                rospy.loginfo('Replacing child1')

            pop.append(child1)
            sco.append(fitness_temp)
            hei.append(height_temp)
            dis.append(distance_temp)
            rospy.loginfo(f'individual {i}: {sco[-1]}')

            fitness_temp, distance_temp, height_temp = fitness(child2)
            while (fitness_temp == -1000):
                num_unfit = num_unfit + 1
                child2 = mutGaussian(best, 0, 1, 0.7)
                fitness_temp, distance_temp, height_temp = fitness(child2)
                rospy.loginfo('Replacing child2')

            pop.append(child2)
            sco.append(fitness_temp)
            hei.append(height_temp)
            dis.append(distance_temp)
            rospy.loginfo(f'individual {i+1}: {sco[-1]}')

            i += 2

        rospy.loginfo('Mutated')
        unfit_count.append(num_unfit)
        num_unfit = 0

        fit = np.array(sco)
        height_arr = np.array(hei)
        dist_arr = np.array(dis)

        unfit_count.append(num_unfit)
        num_unfit = 0

        running_fitness.append(np.max(fit))
        running_height.append(np.max(height_arr))
        running_dist.append(np.max(dist_arr))

        file = open(os.path.join(result_path, f'adv_evolution_1_gen{generation}.txt'),'w') 
        for i in range(len(pop)):
            file.write(str(sco[i])+": ")
            file.write(str(pop[i]))
            file.write("\n \n")
        file.close()
        rospy.loginfo(f'Created "adv_evolution_1_gen"+{generation}+" file')

        file = open(os.path.join(result_path, 'adv_evolution_invalids.txt'),'w')
        file.write(str(unfit_count))
        file.close()

        file = open(os.path.join(result_path, 'adv_evolution_1_fitness.txt'),'w')
        for i in range(len(running_fitness)):
            file.write(str(running_fitness[i])+",")
        file.close()

    file = open(os.path.join(result_path, 'adv_evolution_1_genf.txt'),'w')
    index_best = np.argmax(fit)
    for i in range(len(pop)):
        file.write(str(sco[i])+": ") 
        file.write(str(pop[i]))
        file.write("\n \n")
    file.close()

    rospy.loginfo('Done')

    file = open(os.path.join(result_path, 'adv_evolution_1_fitness.txt'),'w') 
    for i in range(len(running_fitness)):
        file.write(str(running_fitness[i])+",") 
    file.close()
    rospy.loginfo('Done')

    file = open(os.path.join(result_path, 'adv_evolution_1_breakdown.txt'),'w') 
    file.write(str(running_dist)+"\n")
    file.write(str(running_height)+"\n")
    file.close()
    rospy.loginfo('Done')

        

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