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


def run(num_krauss, num_control, scenario):
    sumoBinary = checkBinary('sumo-gui')
    traci.start([sumoBinary, "-c", "lateral_merge.sumocfg" + str(scenario), "--tripinfo-output", "tripinfo.xml", "--random", "--no-warnings", "--error-log", "out.txt"])
    orig_stdout = sys.stdout
    k = int((num_krauss/(num_krauss + num_control)) * 100)
    c = int((num_control/(num_krauss + num_control)) * 100)
    f = open('pso_' + str(num_krauss + num_control) + '_scenario_' + str(scenario) + '_ratio_' + str(k) + "_" + str(c) + '.txt', 'a')
    f.truncate(0)
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
    if scenario == 2:
        k_path = "merge_route_"
        c_path = "route_"
    if scenario == 3:
        k_path = "route_"
        c_path = "route_"
    for k in range(num_krauss):
        route = np.mod(k, 15)
        traci.vehicle.add(str(k), k_path + str(route), typeID="krauss", depart=0, departLane="first",
                          departPos="base", departSpeed='15', arrivalLane='current', arrivalPos='max',
                          arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
    for c in range(num_control):
        route = np.mod(c, 15)
        traci.vehicle.add(str(c + num_krauss), c_path + str(route), typeID="krauss_control", depart=0,
                          departLane="first", departPos="base", departSpeed='15', arrivalLane='current',
                          arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0,
                          personNumber=0)
        car_list.append(str(c + num_krauss))
    previous = ""
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if step >= 500:  # burn in period
            test = traci.inductionloop.getVehicleData("det_1")
            if len(test) == 1:
                a, b, c, d, e = test[0]
                if d != -1.0 and a != previous:
                    total_merge += 1
                    previous = a
            collisions += traci.simulation.getCollidingVehiclesNumber()
            if traci.inductionloop.getLastStepMeanSpeed("det_1") != -1.00:
                merge_speed.append(traci.inductionloop.getLastStepMeanSpeed("det_1"))
            if traci.inductionloop.getLastStepMeanSpeed("det_0") != -1.00:
                loop_speed.append(traci.inductionloop.getLastStepMeanSpeed("det_0"))
        for car in range(len(car_list)):
            if car_list[car] in traci.vehicle.getIDList() and step < 10000:
                traci.vehicle.setSpeedMode(car_list[car], 0)
                traci.vehicle.setLaneChangeMode(car_list[car], 0)
                safe_distance = (traci.vehicle.getSpeed(car_list[car])/10) * traci.vehicle.getLength(car_list[car])
                emergency_stop_distance = 5
                #traci.vehicle.setColor(car_list[car], (255, 0, 0))
                if traci.vehicle.getSpeed(car_list[car]) > traci.lane.getMaxSpeed(traci.vehicle.getLaneID(car_list[car])): #if vehicle > speed limit
                    traci.vehicle.slowDown(car_list[car],traci.lane.getMaxSpeed(traci.vehicle.getLaneID(car_list[car])), 1) #speed up tp speed limit
                leader = traci.vehicle.getLeader(car_list[car], 0.0)
                if leader is not None:
                    if leader[1] > safe_distance:
                        if traci.vehicle.getSpeed(car_list[car]) < traci.lane.getMaxSpeed(traci.vehicle.getLaneID(car_list[car])): #if speed < speed limit
                            traci.vehicle.slowDown(car_list[car], traci.vehicle.getSpeed(car_list[car]) + 1, 1) #increase speed by 1
                        elif traci.vehicle.getSpeed(car_list[car]) > traci.vehicle.getSpeed(leader[0]): #speed greater than leader
                            if traci.vehicle.getSpeed(car_list[car]) >= 1: #insure speed is no lower than 0
                                traci.vehicle.slowDown(car_list[car], traci.vehicle.getSpeed(car_list[car]) - 1, 1) #reduce speed by 1
                        else:
                            pass
                    if leader[1] < safe_distance:
                        if traci.vehicle.getSpeed(car_list[car]) >= traci.vehicle.getSpeed(leader[0]) and leader[1] > emergency_stop_distance: #speed > leader and leader > emergency stop
                            if traci.vehicle.getSpeed(leader[0]) >= 1:
                                traci.vehicle.slowDown(car_list[car], traci.vehicle.getSpeed(leader[0]) - 1, 1) #reduce speed by 1
                        elif traci.vehicle.getSpeed(car_list[car]) >= traci.vehicle.getSpeed(leader[0]) and leader[1] <= emergency_stop_distance: #speed > leader and leader < emergency stop
                            traci.vehicle.slowDown(car_list[car], 0, 1)
                        else:
                            pass
                else:
                    if traci.vehicle.getSpeed(car_list[car]) < traci.lane.getMaxSpeed(traci.vehicle.getLaneID(car_list[car])): #vehicle < speed limit
                        traci.vehicle.slowDown(car_list[car], traci.vehicle.getSpeed(car_list[car]) + 2, 1) #speed up by 2
                polite_gap = 50
                rightLeaders = traci.vehicle.getRightLeaders(car_list[car])
                if len(rightLeaders) != 0 and rightLeaders[0][1] < polite_gap: # if right leaders < polite gap
                    if traci.vehicle.getSpeed(rightLeaders[0][0]) >= 1:
                        traci.vehicle.slowDown(car_list[car],traci.vehicle.getSpeed(rightLeaders[0][0]) - 1, 1 ) #slow down by 1
                merge_lane = traci.lanearea.getLastStepVehicleIDs("det_2")
                merge_gap = 50
                if len(merge_lane) != 0:
                    for vehicle in merge_lane:
                        leftLeaders = traci.vehicle.getLeftLeaders(vehicle)
                        leftFollowers = traci.vehicle.getLeftFollowers(vehicle)
                        if len(leftLeaders) != 0 and len(leftFollowers) != 0: #if both left leaders and followers
                            if leftLeaders[0][1] > merge_gap and leftFollowers[0][1] > merge_gap: #merge gap exists
                                traci.vehicle.changeLane(vehicle, 1, 10.0) #change lanes
                                traci.vehicle.slowDown(vehicle, traci.lane.getMaxSpeed(traci.vehicle.getLaneID(vehicle)), 2) #speed up to speed limit
                            elif leftLeaders[0][1] > merge_gap and leftFollowers[0][1] <= merge_gap:
                                traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(vehicle) + 1, 1)
                            elif leftLeaders[0][1] <= merge_gap and leftFollowers[0][1] > merge_gap:
                                if traci.vehicle.getSpeed(vehicle) >= 1:
                                    traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(vehicle) - 1, 1)
                            else:
                                pass
                        elif len(leftLeaders) != 0 and len(leftFollowers) == 0: # is just left leaders
                            if leftLeaders[0][1] > merge_gap: #merge gap
                                traci.vehicle.changeLane(vehicle, 1, 10.0) #change lanes
                                traci.vehicle.slowDown(vehicle, traci.lane.getMaxSpeed(traci.vehicle.getLaneID(vehicle)), 2) #speed up to speed limit
                            elif leftLeaders[0][1] <= merge_gap:
                                if traci.vehicle.getSpeed(vehicle) >= 1:
                                    traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(vehicle) - 1, 1)
                        elif len(leftLeaders) == 0 and len(leftFollowers) != 0: #if just left followers
                            if leftFollowers[0][1] > merge_gap: #merge gap
                                traci.vehicle.changeLane(vehicle, 1, 10.0) #change lanes
                                traci.vehicle.slowDown(vehicle,
                                traci.lane.getMaxSpeed(traci.vehicle.getLaneID(vehicle)), 2) #speed up to speed limit
                            if leftFollowers[0][1] <= merge_gap:
                                traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(vehicle) + 1, 1)
                        elif len(leftLeaders) == 0 and len(leftFollowers) == 0: #no one around
                            traci.vehicle.changeLane(vehicle, 1, 10.0) #change lanes
                            traci.vehicle.slowDown(vehicle, traci.lane.getMaxSpeed(traci.vehicle.getLaneID(vehicle)), 2) #speed up to speed limit
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


run(0,50,3)