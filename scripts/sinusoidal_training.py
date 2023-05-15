#!/usr/bin/env python3

import rospy
import random
import os
import tf
import math
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from gazebo_services import unload_controllers, load_controllers, call_spawn_model, call_delete_model, start_simulation, pause_simulation

# Defaults
population_size = 50
length = 240
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

def create_individual(length):
    individual = np.random.randint(2, size=length)
    return individual

def first_population(pop_size, length):
    global pop
    
    for i in range(pop_size):
        pop.append(create_individual(length))

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

def fitness(individual_given):
    global height_count
    global height_average
    global pitch_average
    global roll_average
    global unfit
    global rec_count
    global final_disp

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

    individual = decimal_to_binary(individual_given)

    height_count = 1
    height_average = 0
    pitch_average = 0
    roll_average = 0

    initial_time = rospy.get_rostime()
    for i in range(16):        
        hip1.publish(hip_position)
        hip2.publish(hip_position)
        hip3.publish(hip_position)
        hip4.publish(hip_position)


        if (unfit == True):
            performance = -1000
            unfit = False
            unload_controllers()
            call_delete_model('rupert')

            return performance, 0, 0
        
        
        knee1.publish(calc_angle(individual[0], individual[1], individual[2]))
        knee2.publish(calc_angle(individual[3], individual[4], individual[5]))
        knee3.publish(calc_angle(individual[6], individual[7], individual[8]))
        knee4.publish(calc_angle(individual[9], individual[10], individual[11]))
        ankle1.publish(calc_angle(individual[12], individual[13], individual[14]))
        ankle2.publish(calc_angle(individual[15], individual[16], individual[17]))
        ankle3.publish(calc_angle(individual[18], individual[19], individual[20]))
        ankle4.publish(calc_angle(individual[21], individual[22], individual[23]))
        
        rospy.sleep(0.2)

    final_time = rospy.get_rostime()

    performance = (final_position - initial_position) / (final_time.secs - initial_time.secs) + height_average/height_count - roll_average/height_count - pitch_average/height_count
    unload_controllers()
    call_delete_model('rupert')

    if (unfit == True or final_disp < 0):
        performance = -1000
        unfit = False

    return performance, final_position, height_average

def generate_wheel(total_fitness, fitness):
    wheel = []

    for i in range(len(fitness)):
        reps = int(fitness[i]/total_fitness * 100) * 10

        if fitness[i] <= 0 or reps <= 0:
            reps = 1

        wheel.extend([i]*reps)

    return wheel

def select_two(family, family_fitness, family_hei, family_dist):
    # family = [parent1, parent2]
    # family_fitness = [fitness1, fitness2]

    child1, child2 = mate(family[0], family[1])

    fitness_temp, distance_temp, height_temp = fitness(child1)
    family.append(child1)
    family_fitness.append(fitness_temp)
    family_dist.append(distance_temp)
    family_hei.append(height_temp)

    fitness_temp, distance_temp, height_temp = fitness(child2)
    family.append(child2)
    family_fitness.append(fitness_temp)
    family_dist.append(distance_temp)
    family_hei.append(height_temp)

    index1, index2 = np.argpartition(family_fitness, -2)[:2]

    return([family[index1], family[index2]], [family_fitness[index1], family_fitness[index2]],
           [family_hei[index1], family_hei[index2]], [family_dist[index1], family_dist[index2]])

def best_n(n, scores):
    global surv
    global sco
    global hei
    global dis
    global pop

    hei2 = hei
    dis2 = dis

    surv = []
    sco = []
    hei = []
    dis = []

    

def model_state_callback(data):
    global height_count
    global final_position
    global ride_height
    global height_average
    global pitch_average
    global roll_average
    global unfit

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

def mate(individual1, individual2):
    child1 = []
    child2 = []
    for i in range(0,240,10):
        index = random.randrange(10)
        child1 = child1 + individual1[i:i+index].tolist() + individual2[i+index:i+10].tolist()
        child2 = child2 + individual2[i:i+index].tolist() + individual1[i+index:i+10].tolist()
    return np.array(child1), np.array(child2)

def mutation(bitstring, r_mut):
    for i in range(len(bitstring)):
        if np.random.rand() < r_mut:
            bitstring[i] = 1 - bitstring[i]

    return bitstring



def mutate(radiation1, radiation2):
    global pop
    global sco
    global hei
    global dis

    index = 0
    for i in range(int(radiation1*(population_size))):
        index = random.randint(0,(population_size-1))
        new_specimen = pop[index]

        pop[index] = mutation(new_specimen, radiation2)
        sco[index],dis[index],hei[index] = fitness(pop[index])


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
    total_fitness = 0

    generation = 0

    load_controllers()
    unload_controllers()
    call_delete_model('rupert')

    first_population(population_size,length)
    rospy.loginfo(f'Population: {len(pop)} x {len(pop[0])}')

    # Evaluating population fitness and replacing unfit individuals
    for i in range(population_size):
        score, distance_temp, height_temp = fitness(pop[i])
        
    #     while score == -1000:
    #         num_unfit += 1
    #         pop[i] = create_individual(length)
    #         score, distance_temp, height_temp = fitness(pop[i])
    #         rospy.loginfo('Replacing pop 0 individual')

        rospy.loginfo(f'Individual {i}: {score}')
        sco.append(score)
        total_fitness += score
        dis.append(distance_temp)
        hei.append(height_temp)

    rospy.loginfo(f'Sco: {sco}')
    rospy.loginfo(f'Total Fitness: {total_fitness}')
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

        sco = []
        dis = []
        hei = []
        surv = []

        wheel = generate_wheel(total_fitness, fit)
    

        for j in range(0, population_size, 2):
            index1, index2 = random.choices(wheel, k=2)

            while index1 == index2:
                index1, index2 = random.choices(wheel, k=2)
            
            # parent1 = pop[index1]
            # parent2 = pop[index2]
            # fitness1 = fit[index1]
            # fitness2 = fit[index2]

            family = [pop[index1], pop[index2]]
            family_fitness = [fit[index1], fit[index2]]
            family_hei = [height_arr[index1], height_arr[index2]]
            family_dist = [dist_arr[index1], dist_arr[index2]]

            family, family_fitness, family_hei, family_dist = select_two(family, family_fitness, family_hei, family_dist)

            rospy.loginfo(f'Individual {j}: {family_fitness[0]}')
            rospy.loginfo(f'Individual {j+1}: {family_fitness[1]}')

            surv.extend(family)
            sco.extend(family_fitness)
            hei.extend(family_hei)
            dis.extend(family_dist)

        rospy.loginfo('Mutating population')
        if generation<switch_mutation_gen:
            mutate(0.3,0.15)
        else:
            mutate(0.2,0.1)

        rospy.loginfo('Mutated')
    #     unfit_count.append(num_unfit)
    #     num_unfit = 0

        fit = np.array(sco)
        height_arr = np.array(hei)
        dist_arr = np.array(dis)
        total_fitness = np.sum(fit)

        running_fitness.append(np.max(fit))
        running_height.append(np.max(height_arr))
        running_dist.append(np.max(dist_arr))

        file = open(os.path.join(result_path, f'sinusoidal_gen{generation}.txt'),'w') 
        for i in range(len(pop)):
            file.write(str(sco[i])+": ")
            file.write(str(pop[i]))
            file.write("\n \n")
        file.close()
        rospy.loginfo(f'Created "sinusoidal_gen"+{generation}+" file')

        # file = open(os.path.join(result_path, 'evolution_invalids.txt'),'w')
        # file.write(str(unfit_count))
        # file.close()

        file = open(os.path.join(result_path, 'sinusoidal_fitness.txt'),'w')
        for i in range(len(running_fitness)):
            file.write(str(running_fitness[i])+",")
        file.close()

    file = open(os.path.join(result_path, 'sinusoidal_genf.txt'),'w')
    index_best = np.argmax(fit)
    for i in range(len(pop)):
        file.write(str(sco[i])+": ") 
        file.write(str(pop[i]))
        file.write("\n \n")
    file.close()

    rospy.loginfo('Done')

    file = open(os.path.join(result_path, 'sinusoidal_fitness.txt'),'w') 
    for i in range(len(running_fitness)):
        file.write(str(running_fitness[i])+",") 
    file.close()
    rospy.loginfo('Done')

    file = open(os.path.join(result_path, 'sinusoidal_breakdown.txt'),'w') 
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
    rospy.wait_for_service('/gazebo/get_physics_properties')
    rospy.loginfo('STARTING /gazebo/get_physics_properties')
    rospy.wait_for_service('/gazebo/set_physics_properties')
    rospy.loginfo('STARTING /gazebo/set_physics_properties')

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