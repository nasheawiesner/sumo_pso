import os
import sys
import optparse
import numpy as np


#if 'SUMO_HOME' in os.environ:
	#tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
	#sys.path.append(tools)
sys.path.append('/usr/local/opt/sumo/share/sumo/tools/')
#else:
	#sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary
import traci

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


def run(swarm, num_krauss, num_control, scenario):
    sumoBinary = checkBinary('sumo')
    if scenario == 1:
        scenario = "baseline"
    traci.start([sumoBinary, "-c", "merge.sumocfg" + str(scenario), "--tripinfo-output", "tripinfo.xml", "--random", "--no-warnings", "--error-log", "out.txt"])
    orig_stdout = sys.stdout
    #num_krauss = 1
    #num_control = 9
    k = int((num_krauss / (num_krauss + num_control)) * 100)
    c = int((num_control / (num_krauss + num_control)) * 100)
    f = open('pso_' + str(num_krauss + num_control) + '_baseline_' + str(scenario) + '_ratio_' + str(k) + "_" + str(c) + '.txt', 'a')
    sys.stdout = f
    options = get_options()
    step = 0
    sys.stdout.flush()
    total_merge = 0
    merge_speed = []
    loop_speed = []
    collisions = 0
    print(str(num_krauss) + "/" + str(num_control))
    minGap = []
    car_list = []
    k_path = ""
    c_path = ""
    if scenario == "baseline":
        k_path = "merge_route_"
        c_path = "route_"
    if scenario == 3:
        k_path = "route_"
        c_path = "route_"
    for k in range(num_krauss):
        route = np.mod(k, 8)
        traci.vehicle.add(str(k), k_path + str(route), typeID="krauss_control", depart=0, departLane="first",
						  departPos="base", departSpeed='15', arrivalLane='current', arrivalPos='max',
						  arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        car_list.append(str(k + 1))
    for c in range(num_control):
        route = np.mod(c, 8)
        traci.vehicle.add(str(c + num_krauss), c_path + str(route), typeID="krauss_control", depart=0,
						  departLane="first", departPos="base", departSpeed='15', arrivalLane='current',
						  arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0,
						  personNumber=0)
        car_list.append(str((c+1) + num_krauss))
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if step >= 500:  # burn in period
            test = traci.inductionloop.getVehicleData("det_0")
            if len(test) == 1:
                a, b, c, d, e = test[0]
                if d != -1.0:
                    total_merge += 1
            collisions += traci.simulation.getCollidingVehiclesNumber()
            if traci.inductionloop.getLastStepMeanSpeed("det_1") != -1.00:
                merge_speed.append(traci.inductionloop.getLastStepMeanSpeed("det_1"))
            if traci.inductionloop.getLastStepMeanSpeed("det_0") != -1.00:
                loop_speed.append(traci.inductionloop.getLastStepMeanSpeed("det_0"))
        for car in range(len(car_list)):
            if car_list[car] in traci.vehicle.getIDList():
                traci.vehicle.setSpeedMode(car_list[car], 23)
                traci.vehicle.setColor(car_list[car], (0, 0, 255))
                if step.__mod__(200) == 0:
                    traci.vehicle.slowDown(car_list[car], swarm[2], 2)
                else:
                    traci.vehicle.setSpeed(car_list[car], swarm[0])
                if swarm[1] >= 0.1:
                    traci.vehicle.setMinGap(car_list[car], swarm[1])
                else:
                    traci.vehicle.setMinGap(car_list[car], 0.1)
                minGap.append(traci.vehicle.getMinGap(car_list[car]))

        step += 1
    traci.close()
    print("Total Merge: " + str(total_merge))
    print("Total Collisions: " + str(collisions))
    loop_avg = 0
    if len(loop_speed) > 0:
        loop_avg = (sum(loop_speed)) / len(loop_speed)
        print("Loop Speed: " + str(loop_avg))
    merge_avg = 0
    if len(merge_speed) > 0:
        merge_avg = (sum(merge_speed)) / len(merge_speed)
        print("Merge Speed: " + str(merge_avg))
    sys.stdout.flush()
    return total_merge, collisions, loop_avg, merge_avg


