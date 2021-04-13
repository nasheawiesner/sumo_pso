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
import traci.constants as tc

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


def run(num_krauss, num_control, scenario, particles):
    sumoBinary = checkBinary('sumo')
    traci.start([sumoBinary, "-c", "lateral_merge.sumocfg" + str(scenario), "--tripinfo-output", "tripinfo.xml", "--random", "--no-warnings", "--error-log", "out.txt"])#,  "--device.rerouting.threads=4"])
    orig_stdout = sys.stdout
    k = int((num_krauss/(num_krauss + num_control)) * 100)
    c = int((num_control/(num_krauss + num_control)) * 100)
    #f = open('pso_' + str(num_krauss + num_control) + '_scenario_' + str(scenario) + '_ratio_' + str(k) + "_" + str(c) + '.txt', 'a')
    #f.truncate(0)
    #sys.stdout = f
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
    if scenario == 1:
        k_path = "route_"
        c_path = "merge_route_"
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
        #traci.vehicle.subscribe(str(k), (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))
    for c in range(num_control):
        route = np.mod(c, 15)
        traci.vehicle.add(str(c + num_krauss), c_path + str(route), typeID="krauss_control", depart=0,
                          departLane="first", departPos="base", departSpeed='15', arrivalLane='current',
                          arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0,
                          personNumber=0)
        #traci.vehicle.subscribe(str(c), (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))
        car_list.append(str(c + num_krauss))
    previous = ""
    collision_vec = [0] * len(car_list)
    merge_vec = [0] * len(car_list)
    while traci.simulation.getMinExpectedNumber() > 0: #need to tally statistics on each individual vehicle
        traci.simulationStep()
        collisionIDs = ""
        if step >= 500:  # burn in period
            test = traci.inductionloop.getVehicleData("det_1")
            if len(test) == 1:
                a, b, c, d, e = test[0]
                #if a != previous:
                total_merge += 1
                if e == "krauss_control":
                    merge_vec[int(a)-num_krauss] += 1
                #previous = a
            collisions += traci.simulation.getCollidingVehiclesNumber()
            collisionIDs = traci.simulation.getCollidingVehiclesIDList()
            if traci.inductionloop.getLastStepMeanSpeed("det_1") != -1.00:
                merge_speed.append(traci.inductionloop.getLastStepMeanSpeed("det_1"))
            if traci.inductionloop.getLastStepMeanSpeed("det_0") != -1.00:
                loop_speed.append(traci.inductionloop.getLastStepMeanSpeed("det_0"))
        for car in range(len(car_list)):
            if car_list[car] in collisionIDs:
                collision_vec[car] += 1
            if car_list[car] in traci.vehicle.getIDList() and step < 5000:
                traci.vehicle.setSpeedMode(car_list[car], 0)
                traci.vehicle.setLaneChangeMode(car_list[car], 0)
                safe_distance = particles[car][0]
                emergency_stop_distance = particles[car][1]
                traci.vehicle.setColor(car_list[car], (255, 0, 0))
                if traci.vehicle.getSpeed(car_list[car]) > traci.lane.getMaxSpeed(traci.vehicle.getLaneID(car_list[car])): #if vehicle > speed limit
                    traci.vehicle.slowDown(car_list[car],traci.lane.getMaxSpeed(traci.vehicle.getLaneID(car_list[car])), 1) #slow down  to speed limit
                leader = traci.vehicle.getLeader(car_list[car], 0.0)
                if leader is not None:
                    if leader[1] >= safe_distance:
                        if traci.vehicle.getSpeed(car_list[car]) < traci.lane.getMaxSpeed(traci.vehicle.getLaneID(car_list[car])): #if speed < speed limit
                            traci.vehicle.slowDown(car_list[car], traci.vehicle.getSpeed(car_list[car]) + particles[car][2], particles[car][3]) #increase speed by x
                        elif traci.vehicle.getSpeed(car_list[car]) > traci.vehicle.getSpeed(leader[0]): #speed greater than leader
                            if traci.vehicle.getSpeed(car_list[car]) >= particles[car][4]: #insure speed is no lower than 0
                                traci.vehicle.slowDown(car_list[car], traci.vehicle.getSpeed(car_list[car]) - particles[car][4], particles[car][5]) #reduce speed by x
                            else:
                                traci.vehicle.slowDown(car_list[car],0,particles[car][5])
                        else:
                            pass
                    if leader[1] < safe_distance:
                        if traci.vehicle.getSpeed(car_list[car]) >= traci.vehicle.getSpeed(leader[0]) and leader[1] > emergency_stop_distance: #speed > leader and leader > emergency stop
                            if traci.vehicle.getSpeed(leader[0]) >= particles[car][6]:
                                traci.vehicle.slowDown(car_list[car], traci.vehicle.getSpeed(leader[0]) - particles[car][6], particles[car][7]) #reduce speed by x
                            else:
                                traci.vehicle.slowDown(car_list[car],0,particles[car][7])
                        elif traci.vehicle.getSpeed(car_list[car]) >= traci.vehicle.getSpeed(leader[0]) and leader[1] <= emergency_stop_distance: #speed > leader and leader < emergency stop
                            traci.vehicle.slowDown(car_list[car], 0, 1)
                        else:
                            pass
                else: #no leader
                    if traci.vehicle.getSpeed(car_list[car]) < traci.lane.getMaxSpeed(traci.vehicle.getLaneID(car_list[car])): #vehicle < speed limit
                        traci.vehicle.slowDown(car_list[car], traci.vehicle.getSpeed(car_list[car]) + particles[car][8], particles[car][9]) #speed up by x
                polite_gap = particles[car][10]
                rightLeaders = traci.vehicle.getRightLeaders(car_list[car])
                if len(rightLeaders) != 0 and rightLeaders[0][1] < polite_gap: # if right leaders < polite gap
                    if traci.vehicle.getSpeed(rightLeaders[0][0]) >= particles[car][11]:
                        traci.vehicle.slowDown(car_list[car],traci.vehicle.getSpeed(rightLeaders[0][0]) - particles[car][11], particles[car][12]) #slow down by x
                    else:
                        traci.vehicle.slowDown(car_list[car],0,particles[car][12])
                merge_lane = traci.lanearea.getLastStepVehicleIDs("det_2")
                merge_gap = particles[car][13]
                if len(merge_lane) != 0:
                    for vehicle in merge_lane:
                        if vehicle in car_list:
                            leftLeaders = traci.vehicle.getLeftLeaders(vehicle)
                            leftFollowers = traci.vehicle.getLeftFollowers(vehicle)
                            if len(leftLeaders) != 0 and len(leftFollowers) != 0: #if both left leaders and followers
                                if leftLeaders[0][1] > merge_gap and leftFollowers[0][1] > merge_gap: #merge gap exists
                                    traci.vehicle.changeLane(vehicle, 1, 10.0) #change lanes
                                    if traci.vehicle.getLeader(vehicle, 0.0) is not None: #if there is a leader
                                        leader = traci.vehicle.getLeader(vehicle, 0.0)
                                        traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(leader[0]), particles[car][14]) #speed up to leader speed
                                    else:
                                        traci.vehicle.slowDown(vehicle,traci.lane.getMaxSpeed(traci.vehicle.getLaneID(vehicle)),particles[car][14])  # speed up to speed limit
                                elif leftLeaders[0][1] > merge_gap and leftFollowers[0][1] <= merge_gap:
                                    traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(vehicle) + particles[car][15], particles[car][16]) #speed up to merge
                                elif leftLeaders[0][1] <= merge_gap and leftFollowers[0][1] > merge_gap:
                                    if traci.vehicle.getSpeed(vehicle) >= particles[car][17]:
                                        traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(vehicle) - particles[car][17], particles[car][18]) #slow down to merge
                                    else:
                                        traci.vehicle.slowDown(vehicle,0,particles[car][18])
                                else:
                                    pass
                            elif len(leftLeaders) != 0 and len(leftFollowers) == 0: # is just left leaders
                                if leftLeaders[0][1] > merge_gap: #merge gap
                                    traci.vehicle.changeLane(vehicle, 1, 10.0) #change lanes
                                    if traci.vehicle.getLeader(vehicle, 0.0) is not None:  # if there is a leader
                                        leader = traci.vehicle.getLeader(vehicle, 0.0)
                                        traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(leader[0]),particles[car][14])  # speed up to leader speed
                                    else:
                                        traci.vehicle.slowDown(vehicle,traci.lane.getMaxSpeed(traci.vehicle.getLaneID(vehicle)),particles[car][14])  # speed up to speed limit
                                elif leftLeaders[0][1] <= merge_gap:
                                    if traci.vehicle.getSpeed(vehicle) >= particles[car][17]:
                                        traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(vehicle) - particles[car][17], particles[car][18]) #slow down to merge
                                    else:
                                        traci.vehicle.slowDown(vehicle,0,particles[car][18])
                            elif len(leftLeaders) == 0 and len(leftFollowers) != 0: #if just left followers
                                if leftFollowers[0][1] > merge_gap: #merge gap
                                    traci.vehicle.changeLane(vehicle, 1, 10.0) #change lanes
                                    traci.vehicle.slowDown(vehicle,
                                    traci.lane.getMaxSpeed(traci.vehicle.getLaneID(vehicle)), particles[car][14]) #speed up to speed limit
                                if leftFollowers[0][1] <= merge_gap:
                                    traci.vehicle.slowDown(vehicle, traci.vehicle.getSpeed(vehicle) + particles[car][15], particles[car][16]) #speed up to merge
                            elif len(leftLeaders) == 0 and len(leftFollowers) == 0: #no one around
                                traci.vehicle.changeLane(vehicle, 1, 10.0) #change lanes
                                traci.vehicle.slowDown(vehicle, traci.lane.getMaxSpeed(traci.vehicle.getLaneID(vehicle)), particles[car][14]) #speed up to speed limit
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
    return total_merge, collisions, loop_avg, merge_avg, merge_vec, collision_vec


#run(0,50,3,[])