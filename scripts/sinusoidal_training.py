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
from gazebo_services import unload_controllers, load_controllers, call_spawn_model, call_delete_model

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

def create_individual(length):
    individual = []
    
    
    return individual


def first_population(pop_size, length):
	global pop

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

    except:
        pass

def mate(individual1, individual2):
    child1, child2 = [], []

    return child1, child2

def mutate(radiation1, radiation2):
    global pop
    global sco
    global hei
    global dis

    new_specimen = []
    index = random.randint()

    pop[index] = new_specimen
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

    generation = 0

    load_controllers()
    unload_controllers()
    call_delete_model('rupert')

    first_population(population_size,length)
    rospy.loginfo(f'Population: {len(pop)} x {len(pop[0])}')

    # Evaluating population fitness and replacing unfit individuals
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

        best_n(population_size//2, fit)

        for j in range(len(pop)):
            if(j%2==0):
                partner1 = random.randint(0, len(pop)-1)
                partner2 = random.randint(0, len(pop)-1)

                child1, child2 = mate(pop[partner1], pop[partner2])
                
                fitness_temp, distance_temp, height_temp = fitness(child1)
                while(fitness_temp == -1000):
                    num_unfit = num_unfit + 1
                    child1 = create_individual(length)
                    fitness_temp, distance_temp, height_temp = fitness(child1)
                    rospy.loginfo('Replacing child1')
                pop.append(child1)
                sco.append(fitness_temp)
                hei.append(height_temp)
                dis.append(distance_temp)
                rospy.loginfo(f'individual {j}: {sco[-1]}')

                fitness_temp, distance_temp, height_temp = fitness(child2)
                while(fitness_temp == -1000):
                    num_unfit = num_unfit + 1
                    child2 = create_individual(length)
                    fitness_temp, distance_temp, height_temp = fitness(child2)
                    rospy.loginfo('Replacing child2')
                pop.append(child2)
                sco.append(fitness_temp)
                hei.append(height_temp)
                dis.append(distance_temp)
                rospy.loginfo(f'individual {j+1}: {sco[-1]}')

        if generation<switch_mutation_gen:
            mutate(0.3,0.5)
        else:
            mutate(0.2,0.3)

        rospy.loginfo('Mutated')
        unfit_count.append(num_unfit)
        num_unfit = 0

        fit = np.array(sco)
        height_arr = np.array(hei)
        dist_arr = np.array(dis)

        running_fitness.append(np.max(fit))
        running_height.append(np.max(height_arr))
        running_dist.append(np.max(dist_arr))

        file = open(os.path.join(result_path, f'evolution_1_gen{generation}.txt'),'w') 
        for i in range(len(pop)):
            file.write(str(sco[i])+": ")
            file.write(str(pop[i]))
            file.write("\n \n")
        file.close()
        rospy.loginfo(f'Created "evolution_1_gen"+{generation}+" file')

        file = open(os.path.join(result_path, 'evolution_invalids.txt'),'w')
        file.write(str(unfit_count))
        file.close()

        file = open(os.path.join(result_path, 'evolution_1_fitness.txt'),'w')
        for i in range(len(running_fitness)):
            file.write(str(running_fitness[i])+",")
        file.close()

    file = open(os.path.join(result_path, 'evolution_1_genf.txt'),'w')
    index_best = np.argmax(fit)
    for i in range(len(pop)):
        file.write(str(sco[i])+": ") 
        file.write(str(pop[i]))
        file.write("\n \n")
    file.close()

    rospy.loginfo('Done')

    file = open(os.path.join(result_path, 'evolution_1_fitness.txt'),'w') 
    for i in range(len(running_fitness)):
        file.write(str(running_fitness[i])+",") 
    file.close()
    rospy.loginfo('Done')

    file = open(os.path.join(result_path, 'evolution_1_breakdown.txt'),'w') 
    file.write(str(running_dist)+"\n")
    file.write(str(running_height)+"\n")
    file.close()
    rospy.loginfo('Done')

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