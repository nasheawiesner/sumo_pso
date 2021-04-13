from CPSO import *
from merge_co_pso import *
import copy

population_vec = []
num_iterations = 250

tot_merge = 0
tot_coll = 0
merge_speed = 0
loop_speed = 0

individual_merges = []
individual_collisions = []
tot_merge_vec = []
tot_coll_vec = []
ls_vec = []
ms_vec = []
individual_merges_vec = []
individual_collisions_vec = []
fit = 0
fit_vec = []
particle_best_fitness_vec = []
global_best_fitness_vec = []
particle_best_vec = []
global_best_vec = []
population_velocity_vec = []
population = []
population_velocity = []

def coordinate(num_pso, num_krauss, scenario):
    #file output
    orig_stdout = sys.stdout
    print("(" + str(num_pso) + "," + str(num_krauss) + "," + str(scenario) + ")")
    k = int((num_krauss / (num_krauss + num_pso)) * 100)
    c = int((num_pso / (num_krauss + num_pso)) * 100)
    f = open('pso_' + str(num_krauss + num_pso) + '_scenario_' + str(scenario) + '_ratio_' + str(c) + "_" + str(
        k) + '.txt', 'a')
    f.truncate(0)
    sys.stdout = f
    #pso initialization
    for pso in range(num_pso):
        population, population_velocity = initialize()
        copy = np.copy(population)
        particle_best_vec.append([0.0] * len(population))
        particle_best_fitness_vec.append([0] * len(population))
        global_best_vec.append([0])
        global_best_fitness_vec.append([0])
        population_vec.append(copy)
        population_velocity_vec.append(population_velocity)
    run_sumo(True, num_pso, num_krauss, scenario)

    #run pso
    for iter in range(num_iterations):
        print("Iteration " + str(iter))
        for pso in range(num_pso):
            #update particles and particle velocities
            population_vec[pso], population_velocity_vec[pso] = velocity_particle_update(population_vec[pso], population_velocity_vec[pso], particle_best_vec[pso], global_best_vec[pso][-1])
        run_sumo(False, num_pso, num_krauss, scenario)

    print("fitness vector")
    print(fit_vec)
    print("population vector")
    print(population_vec)
    print("particle best fitness")
    print(particle_best_fitness_vec)
    print("particle best")
    print(particle_best_vec)
    print("global best fitness")
    print(global_best_fitness_vec)
    print("global best")
    print(global_best_vec)
    print("total merges")
    print(tot_merge_vec)
    print("total collisions")
    print(tot_coll_vec)
    print("average loop speeds")
    print(ls_vec)
    print("average merge speeds")
    print(ms_vec)
    print("individual merges")
    print(individual_merges_vec)
    print("individual collisions")
    print(individual_collisions_vec)

#run simulation
def run_sumo(initial, num_pso, num_krauss, scenario):
    for s in range(swarm_size):
        sim_vec = []
        for p in range(len(population_vec)):
            sim_vec.append(population_vec[p][s])

        #run simulation
        tot_merge, tot_coll, loop_speed, merge_speed, individual_merges, individual_collisions = run(num_krauss, num_pso, scenario, sim_vec)
        tot_merge_vec.append(tot_merge)
        tot_coll_vec.append(tot_coll)
        ls_vec.append(loop_speed)
        ms_vec.append(merge_speed)
        individual_merges_vec.append(individual_merges)
        individual_collisions_vec.append(individual_collisions)
        calc_fitness_best(initial, num_pso, s, individual_merges, individual_collisions)

#caluclates local and global best
def calc_fitness_best(initial, num_pso, s, merges, collisions):
    for pso in range(num_pso):
        fit = fitness(merges[pso], collisions[pso])
        fit_vec.append(fit)
        if initial is True: #initialize particle best to first particle value and fitness
            particle_best_fitness_vec[pso][s] = fit
            temp = np.copy(population_vec[pso][s])
            particle_best_vec[pso][s] = temp
            if s == 0: #if on first swarm
                global_best_fitness_vec[pso][s] = fit
                global_best_vec[pso][s] = population_vec[pso][s]
        else:
            if fit > particle_best_fitness_vec[pso][s]:
                particle_best_fitness_vec[pso][s] = fit
                temp = np.copy(population_vec[pso][s])
                particle_best_vec[pso][s] = temp
        if fit > global_best_fitness_vec[pso][-1]:
            global_best_fitness_vec[pso].append(fit)
            global_best_vec[pso].append(population_vec[pso][s])
        else:
            #copy global best to the end of array to track changes
            global_best_fitness_vec[pso].append(global_best_fitness_vec[pso][-1])
            global_best_vec[pso].append(global_best_vec[pso][-1])
coordinate(1,99,3)