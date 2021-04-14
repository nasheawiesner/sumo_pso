
import numpy as np
import time as time
from merge_co_pso import run
import statistics
import math
import csv
import copy

swarm_size = 20  # number of the swarm particles
iterations = 50  # maximum number of iterations
inertia = 0.5  # inertia of a particle
dimensions = 19  # number of values the PSO will optimize
local_weight = 0.7  # weighted factor for the particles historical best
global_weight = 0.7  # weighted factor for the the global best
max_velocity = 1  # the highest velocity allowed for a particle
step_size = 1  # step size for updating each particle, or how far a particle
                # travels before its velocity is readjusted
max_distance = 50.0
max_emergency_distance = 15.0
minSpeed = 0.0
maxSpeed = 30.0
speed_change = 30
speed_change_time = 10
minSlowDown = 5.0
maxSlowDown = 25.0
minLoopSpeed = 10.0
maxLoopSpeed = 25.0
minMergeSpeed = 0.0
maxMergeSpeed = 15.0
merge_vec = []
coll_vec = []
loop_speed_vec = []
merge_speed_vec = []
max_merges = 0
min_collisions = 1000


def fitness(merge, collisions):
    if merge == 0.0 and collisions > 0.0:
        fitness = 0 - math.log((collisions ** 2))
    elif collisions == 0.0 and merge > 0.0:
        fitness = math.log(merge)
    elif merge == 0.0 and collisions == 0.0:
        fitness = 0.0
    else:
        fitness = (math.log(merge)) - math.log((collisions ** 2))
    return fitness

def initialize():
    # set the x,y location of the particle's (initial guess)
    particle_location = []  # np.zeros((swarm_size, dimensions))#np.random.uniform(0.0, 25.0, (swarm_size,dimensions))

    for i in range(swarm_size):
        vehicle = []
        vehicle.append(np.random.uniform(0.0, max_distance))  #safe distance
        vehicle.append(np.random.uniform(0.0, max_emergency_distance))    #emergency distance
        vehicle.append(np.random.uniform(0.0, speed_change))  #safe leader speed up
        vehicle.append(np.random.uniform(0.0, speed_change_time))  # safe leader speed up time
        vehicle.append(np.random.uniform(0.0, speed_change))  # safe leader slow down
        vehicle.append(np.random.uniform(0.0, speed_change_time))  # safe leader slow down time
        vehicle.append(np.random.uniform(0.0, speed_change))  # unsafe leader slow down
        vehicle.append(np.random.uniform(0.0, speed_change_time))  # unsafe leader slow down time
        vehicle.append(np.random.uniform(0.0, speed_change))  # no leader speed up
        vehicle.append(np.random.uniform(0.0, speed_change_time))  # no leader speed up time
        vehicle.append(np.random.uniform(0.0, max_distance))  # polite gap
        vehicle.append(np.random.uniform(0.0, speed_change))  #polite merge slow down
        vehicle.append(np.random.uniform(0.0, speed_change_time))  # polite merge slow down time
        vehicle.append(np.random.uniform(0.0, max_distance))  # merge gap
        vehicle.append(np.random.uniform(0.0, speed_change))  # speed up after merge time
        vehicle.append(np.random.uniform(0.0, speed_change))  # speed up to merge
        vehicle.append(np.random.uniform(0.0, speed_change_time))  # speed up to merge time
        vehicle.append(np.random.uniform(0.0, speed_change))  # slow down to merge
        vehicle.append(np.random.uniform(0.0, speed_change_time))  # slow down to merge time
        particle_location.append(vehicle)
    particle_location = np.asarray(particle_location)
    # set the initial velocity of the particles in each direction
    particle_velocity = np.random.rand(swarm_size, dimensions)

    return particle_location, particle_velocity


def velocity_particle_update(particle_location, particle_velocity, particle_best_location, global_best_location):
    for particle_i in range(swarm_size):  # for each particle
        for dimension_i in range(dimensions):  # for each dimension

            # generate 2 random numbers between 0 and 1
            u = np.random.rand(dimensions)

            # calculate the error between the particle's best location and its current location
            error_particle_best = particle_best_location[particle_i][dimension_i] - \
                                      particle_location[particle_i][dimension_i]
            # calculate the error between the global best location and the particle's current location
            error_global_best = global_best_location[dimension_i] - \
                                    particle_location[particle_i][dimension_i]

            # update the velocity vector in a given dimension
            v_new = inertia * particle_velocity[particle_i][dimension_i] + \
                        local_weight * u[0] * error_particle_best + \
                        global_weight * u[1] * error_global_best

            # bound a particle's velocity to the maximum value set above
            if v_new < -max_velocity:
                    v_new = -max_velocity
            elif v_new > max_velocity:
                    v_new = max_velocity

            # update the particle location
            particle_location[particle_i][dimension_i] = particle_location[particle_i][dimension_i] + \
                                                             v_new * step_size
            if particle_location[particle_i][dimension_i] < 0.0:
                particle_location[particle_i][dimension_i] = 0.0

            # update the particle velocity
            particle_velocity[particle_i][dimension_i] = v_new
    return particle_location, particle_velocity





