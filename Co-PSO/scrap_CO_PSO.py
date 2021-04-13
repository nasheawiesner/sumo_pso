
import numpy as np
import time as time
import matplotlib.pyplot as plt
import matplotlib as mpl
from merge_pso import run
import statistics
import math
import csv

plt.close('all')
minSpeed = 0.0
maxSpeed = 25.0
minMinGap = 0.1
maxMinGap = 5.0
minSlowDown = 5
maxSlowDown = 25
minLoopSpeed = 10.0
maxLoopSpeed = 25.0
minMergeSpeed = 0.0
maxMergeSpeed = 15.0
merge_vec = []
coll_vec = []
loop_speed_vec = []
merge_speed_vec = []
swarm_size = 10                       # number of the swarm particles
iterations = 50                     # maximum number of iterations
inertia = 0.5                        # inertia of a particle
dimensions = 3                       # number of values the PSO will optimize
local_weight = 2                     # weighted factor for the particles historical best
global_weight = 2                   # weighted factor for the the global best
max_velocity = 1                     # the highest velocity allowed for a particle
step_size = 1                        # step size for updating each particle, or how far a particle
                                         # travels before its velocity is readjusted

max_merges = 0
min_collisions = 1000
swarm_merge_avg = []
swarm_coll_avg = []
swarm_ls_avg = []
swarm_ms_avg = []
swarm_fit_avg = []
gb_merge = []
gb_coll = []
gb_ls = []
gb_ms = []
gb_fitness = []
gb_change = 0


def Function(swarm, max_merge, min_coll, krauss, control, s):
    merge, collisions, loop_speed, merge_speed = run(swarm, krauss, control, s)
    merge_vec.append(merge)
    coll_vec.append(collisions)
    loop_speed_vec.append(loop_speed)
    merge_speed_vec.append(merge_speed)
    fitness = math.log(merge) - (collisions**3)
    if merge >= max_merge and collisions <= min_coll:
        min_coll = collisions
        max_merge = merge
    return fitness, max_merge, min_coll, merge, collisions, loop_speed, merge_speed
def initialize(num_krauss, num_pso, scn):
                                         
    # set the x,y location of the particle's (initial guess)
    particle_location = []#np.zeros((swarm_size, dimensions))#np.random.uniform(0.0, 25.0, (swarm_size,dimensions))

    for i in range(swarm_size):
        vehicle = []
        vehicle.append(np.random.uniform(minSpeed, maxSpeed))
        vehicle.append(np.random.uniform(minMinGap, maxMinGap))
        vehicle.append(np.random.uniform(minSlowDown, maxSlowDown))
        particle_location.append(vehicle)
    particle_location = np.asarray(particle_location)
    # set the initial velocity of the particles in each direction
    particle_velocity = np.random.rand(swarm_size,dimensions)

    # solve the function for the particle's locations and save as their local best

    particle_best_value = []
    swarm_merge = []
    swarm_coll = []
    swarm_ls = []
    swarm_ms = []
    swarm_fitness = []
    for p in particle_location:
        particle_value, max_merges, min_collisions, m, c, ls, ms = Function(p, max_merges, min_collisions, num_krauss, num_pso, scn)   #particle_location[:,0],particle_location[:,1], particle_location[:,2],
        print(p)
        print("fitness: " + str(particle_value))
        particle_best_value.append(particle_value)
        swarm_merge.append(m)
        swarm_coll.append(c)
        swarm_ls.append(ls)
        swarm_ms.append(ms)
        swarm_fitness.append(particle_value)
    particle_best_location = np.copy(particle_location)
    swarm_merge_avg.append(statistics.mean(swarm_merge))
    swarm_coll_avg.append(statistics.mean(swarm_coll))
    swarm_ls_avg.append(statistics.mean(swarm_ls))
    swarm_ms_avg.append(statistics.mean(swarm_ms))
    swarm_fit_avg.append(statistics.mean(swarm_fitness))
    # find the global best location of the initial guess and update the global best location
    index = np.where(particle_best_value == np.max(particle_best_value))
    index = index[0][0]
    global_best_value = np.max(particle_best_value)
    global_best_location = particle_location[np.argmax(particle_best_value)].copy()
    gb_merge.append(swarm_merge[index])
    gb_merge_temp = swarm_merge[index]
    gb_coll.append(swarm_coll[index])
    gb_coll_temp = swarm_coll[index]
    gb_ls.append(swarm_ls[index])
    gb_ls_temp = swarm_ls[index]
    gb_ms.append(swarm_ms[index])
    gb_ms_temp = swarm_ms[index]
    gb_fitness.append(global_best_value)
    gb_fit_temp = global_best_value
    return particle_location
    # create empty lists that are updated during the processes and used only for 
    # plotting the final results

    # best_value = []                  # for the best fitting value
    # best_locaion = []                # for the location of the best fitting value
    # iteration_value_best = []        # for the best fitting value of the iteration
    # iteration_locaion_best = []      # for the location of the best fitting value of the iteration
    # iteration_value = []             # for the values of the iteration
    # iteration_locaion = []           # for the locations of the iteration

    #%% Run PSO code
def pso(particle_location):
    for iteration_i in range(iterations): # for each iteration
        print("Iteration " + str(iteration_i))
        swarm_merge = []
        swarm_coll = []
        swarm_ls = []
        swarm_ms = []
        swarm_fitness = []
        for particle_i in range(swarm_size): # for each particle
            for dimension_i in range(dimensions): # for each dimension 
            
                # generate 2 random numbers between 0 and 1
                u = np.random.rand(dimensions) 

                # calculate the error between the particle's best location and its current location
                error_particle_best = particle_best_location[particle_i,dimension_i] - \
                    particle_location[particle_i,dimension_i]
                # calculate the error between the global best location and the particle's current location
                error_global_best = global_best_location[dimension_i] - \
                    particle_location[particle_i,dimension_i]
                
                # update the velocity vector in a given dimension            
                v_new = inertia*particle_velocity[particle_i,dimension_i] + \
                    local_weight*u[0]*error_particle_best + \
                    global_weight*u[1]*error_global_best

                # bound a particle's velocity to the maximum value set above       
                if v_new < -max_velocity:
                    v_new = -max_velocity
                elif v_new > max_velocity:
                    v_new = max_velocity
                
                # update the particle location
                particle_location[particle_i,dimension_i] = particle_location[particle_i,dimension_i] + \
                    v_new*step_size
                
                # update the particle velocity
                particle_velocity[particle_i,dimension_i] = v_new
            
            # for the new location, check if this is a new local or global best
            v, max_merges, min_collisions, m, c, ls, ms = Function(particle_location[particle_i], max_merges, min_collisions, num_krauss, num_pso, scn)   #particle_location[particle_i,0], particle_location[particle_i,1], particle_location[particle_i,2],
            swarm_merge.append(m)
            swarm_coll.append(c)
            swarm_ls.append(ls)
            swarm_ms.append(ms)
            swarm_fitness.append(v)
            # update if its a new local best
            if v > particle_best_value[particle_i]:
                particle_best_value[particle_i]=v
                particle_best_location[particle_i,:] = particle_location[particle_i,:].copy()
            # update if its a new global best
            if v > global_best_value:
                global_best_value=v
                global_best_location = particle_location[particle_i,:].copy()
                gb_merge_temp = m
                gb_coll_temp = c
                gb_ls_temp = ls
                gb_ms_temp = ms
                gb_fit_temp = v
                gb_change += 1
            print("Particle Best for fitness:" + str(global_best_value))
            # m, c, ls, ms = simulate(particle_best_location[0][0], particle_best_location[0][1], particle_best_location[0][2])
        # print the current best location to the console
        gb_merge.append(gb_merge_temp)
        gb_coll.append(gb_coll_temp)
        gb_ls.append(gb_ls_temp)
        gb_ms.append(gb_ms_temp)
        gb_fitness.append(gb_fit_temp)
        swarm_merge_avg.append(statistics.mean(swarm_merge))
        swarm_coll_avg.append(statistics.mean(swarm_coll))
        swarm_ls_avg.append(statistics.mean(swarm_ls))
        swarm_ms_avg.append(statistics.mean(swarm_ms))
        swarm_fit_avg.append(statistics.mean(swarm_fitness))
        print('solution for speed='+'%.2f' % global_best_location[0]+', min gap='+'%.2f' % global_best_location[1] + ", max merges: " + str(max_merges) + ", min collisions: " + str(min_collisions))

        # update the lists
        # for p in particle_location:
        #     v, max_merges, min_collisions, m, c, ls, ms = Function(p.copy(), max_merges, min_collisions)    #particle_location[:,0].copy(), particle_location[:,1].copy(), particle_location[:,2].copy(),
        #     iteration_value.append(v)
        #     iteration_locaion.append(p.copy())
        #     iteration_value_best.append(np.max(v))
        #     iteration_locaion_best.append(p[np.argmax(v)])
        # best_value.append(global_best_value)
        # best_locaion.append(global_best_location.copy())

    #print("Global Best:")
    # m, c, ls, ms = simulate(global_best_location[0], global_best_location[1], global_best_location[2])
    print("Merges")
    print(merge_vec)
    print("Collisions")
    print(coll_vec)
    print("Loop Speeds")
    print(loop_speed_vec)
    print("Merge Speeds")
    print(merge_speed_vec)
    print("Swarm Average Merges")
    print(swarm_merge_avg)
    print("Swarm Average Collisions")
    print(swarm_coll_avg)
    print("Swarm Average Loop Speeds")
    print(swarm_ls_avg)
    print("Swarm Average Merge Speeds")
    print(swarm_ms_avg)
    print("Swarm Average Fitness")
    print(swarm_fit_avg)
    print("GB Merges")
    print(gb_merge)
    print("GB Collisions")
    print(gb_coll)
    print("GB Loop Speeds")
    print(gb_ls)
    print("GB Merge Speeds")
    print(gb_ms)
    print("GB Fitness")
    print(gb_fitness)
    print("GB Changes")
    print(gb_change)
    k = int((num_krauss / (num_krauss + num_pso)) * 100)
    c = int((num_pso / (num_krauss + num_pso)) * 100)
    with open('pso_' + str(num_krauss + num_pso) + '_scenario_' + str(scn) + '_ratio_' + str(k) + "_" + str(
            c) + '.csv', mode='w') as file:
        file_writer = csv.writer(file, delimiter=',')
        file_writer.writerow(["Swarm Average Merges", "GB Merges", "Swarm Average Collisions", "GB Collisions",
                              "Swarm Average Loop Speeds", "GB Loop Speeds", "Swarm Average Merge Speeds",
                              "GB Merge Speeds"])
        file_writer.writerow(swarm_merge_avg)
        file_writer.writerow(gb_merge)
        file_writer.writerow(swarm_coll_avg)
        file_writer.writerow(gb_coll)
        file_writer.writerow(swarm_ls_avg)
        file_writer.writerow(gb_ls)
        file_writer.writerow(swarm_ms_avg)
        file_writer.writerow(gb_ms)

#pso(1, 9, 1)
#pso(5, 5, 1)
#pso(9, 1, 1)

pso(2, 18, 1)
pso(10, 10, 1)
pso(18, 2, 1)

pso(5, 45, 1)
pso(25, 25, 1)
pso(45, 5, 1)

#pso(10, 90, 1)
#pso(50, 50, 1)
#pso(90, 10, 1)
    # #%% Plot the final results
    # plt.figure(figsize=(5,5))
    # plt.grid('on')
    # plt.rc('axes', axisbelow=True)
    # plt.scatter(13.9,0.1,100,marker='*',facecolors='k', edgecolors='k')
    # #plt.text(10,21,'optimal solution',horizontalalignment='center')
    # for i in range(len(iteration_locaion)):
    #     plt.scatter(iteration_locaion[i][:,0],iteration_locaion[i][:,1],10,marker='x')
    #     plt.scatter(best_locaion[i][0],best_locaion[i][1],50,marker='o',facecolors='none',edgecolors='k',linewidths=0.4)
    #     plt.text(best_locaion[i][0]+0.1,best_locaion[i][1]+0.1,str(i),fontsize=8)
    # plt.xlim(-1,30)
    # plt.ylim(-1,10)
    # plt.xlabel('$x$')
    # plt.ylabel('$y$')
    # plt.title('2-dimensional particle swarm optimization')
    # plt.savefig('results',dpi=300)

# def simulate(speed, gap, slowDown):
#     pbest = []
#     print("speed: " +str(speed))
#     print("gap: " + str(gap))
#     print("slow down: " + str(slowDown))
#     pbest.append(speed)
#     pbest.append(gap)
#     pbest.append(slowDown)
#     pbest = np.asarray(pbest)
#     merge, collisions, l_speed, m_speed = run(pbest)
#     merge_vec.append(merge)
#     coll_vec.append(collisions)
#     loop_speed_vec.append(l_speed)
#     merge_speed_vec.append(m_speed)
#     print("best merges: " + str(merge))
#     print("best collisions: " + str(collisions))
#     return merge, collisions, l_speed, m_speed



