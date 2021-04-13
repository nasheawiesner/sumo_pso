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


def run(num_krauss, num_control):
	step = 0
	sys.stdout.flush()
	total_merge = 0
	merge_speed = []
	loop_speed = []
	collisions = 0
	print(str(num_krauss) + "/" + str(num_control))
	car_list = []
	for k in range(num_krauss):
		route = np.mod(k,8)
		traci.vehicle.add(str(k), "merge_route_" + str(route), typeID="krauss", depart=0, departLane="first", departPos="base", departSpeed='15', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
	for c in range(num_control):
		route = np.mod(c,8)
		traci.vehicle.add(str(c + num_krauss), "merge_route_" + str(route), typeID="krauss_control", depart=0, departLane="first", departPos="base", departSpeed='15', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
		car_list.append(str(c + num_krauss))
	while traci.simulation.getMinExpectedNumber() > 0:
		traci.simulationStep()
		#print(step)
		if step >= 500: #burn in period
			#total_merge += traci.multientryexit.getLastStepVehicleNumber("merge_det")
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
				traci.vehicle.setColor(car_list[car], (0,0,255))
				if step.__mod__(200) == 0:
					traci.vehicle.slowDown(car_list[car], 10, 2)
				else:
					traci.vehicle.setSpeed(car_list[car], 25)



		step += 1
	traci.close()
	print("Total Merge: " + str(total_merge))
	print("Total Collisions: " + str(collisions))
	print("Merge Speed: " + str((sum(merge_speed))/len(merge_speed)))
	print("Loop Speed: " + str((sum(loop_speed)) / len(loop_speed)))
	sys.stdout.flush()
	return total_merge, collisions, (sum(merge_speed))/len(merge_speed), (sum(loop_speed))/len(loop_speed)

if __name__ == "__main__":
	orig_stdout = sys.stdout
	krauss = 1
	auto = 9
	f = open(str(krauss + auto) + '_scenario_3_test_2.txt', 'w')
	sys.stdout = f
	options = get_options()

	if options.nogui:
		sumoBinary = checkBinary('sumo')
	else:
		sumoBinary = checkBinary('sumo-gui')


		merge_arr = []
		coll_arr = []
		m_speed_arr = []
		l_speed_arr = []
		num_sims = 10
		for sim in range(num_sims):
			print(sim + 1)
			traci.start([sumoBinary, "-c", "merge.sumocfg", "--tripinfo-output", "tripinfo.xml", "--random", "--no-warnings", "--error-log", "out.txt"])
			merge, coll, avg_m_speed, avg_l_speed = run(krauss, auto)
			merge_arr.append(merge)
			coll_arr.append(coll)
			m_speed_arr.append(avg_m_speed)
			l_speed_arr.append(avg_l_speed)
		print("\n\nMin Number of Merges: " + str(min(merge_arr)))
		print("Average Number of Merges: " + str((sum(merge_arr))/len(merge_arr)))
		print("Max Number of Merges: " + str(max(merge_arr)))
		print("\nMin Number of Collisions: " + str(min(coll_arr)))
		print("Average Number of Collisions: " + str((sum(coll_arr)) / len(coll_arr)))
		print("Max Number of Collisions: " + str(max(coll_arr)))
		print("\nMin Merge Speed: " + str(min(m_speed_arr)))
		print("Average Merge Speed: " + str((sum(m_speed_arr)) / len(m_speed_arr)))
		print("Max Merge Speed: " + str(max(m_speed_arr)))
		print("\nMin Loop Speed: " + str(min(l_speed_arr)))
		print("Average Loop Speed: " + str((sum(l_speed_arr)) / len(l_speed_arr)))
		print("Max Loop Speed: " + str(max(l_speed_arr)))
