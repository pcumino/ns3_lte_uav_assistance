/* -*- Mode:C++; c-file-style:gnu; indent-tabs-mode:nil; -*- */
/*	Copyright (c) 2019

* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation;
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* Author: Pedro Luiz Magalhães Cumino <pcumino@ua.pt>

*/


#include <algorithm>
#include <array>
#include <bits/stdc++.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split
#include <boost/bind.hpp>
#include <chrono>
#include <cstdlib>
#include <float.h>
#include <fstream>
#include <iomanip>
#include <iostream>   // std::cout
#include <limits>
#include <list>
#include <map>
#include <math.h>
#include <memory>
#include <ns3/applications-module.h>
#include <ns3/basic-energy-source-helper.h>
#include <ns3/basic-energy-source.h>
#include <ns3/boolean.h>
#include <ns3/buildings-module.h>
#include <ns3/config-store-module.h>
#include <ns3/config-store.h>
#include <ns3/core-module.h>
#include <ns3/csma-helper.h>
#include <ns3/device-energy-model-container.h>
#include <ns3/double.h>
#include <ns3/energy-module.h>
#include <ns3/energy-source-container.h>
#include <ns3/enum.h>
#include <ns3/epc-helper.h>
#include <ns3/flow-monitor-module.h>
#include <ns3/gnuplot.h> //gnuplot
#include <ns3/hybrid-buildings-propagation-loss-model.h>
#include <ns3/internet-apps-module.h>
#include <ns3/internet-module.h>
#include <ns3/ipv4-global-routing-helper.h>
#include <ns3/li-ion-energy-source-helper.h>
#include <ns3/li-ion-energy-source.h>
#include <ns3/log.h>
#include <ns3/lte-helper.h>
#include <ns3/lte-module.h>
#include <ns3/lte-ue-net-device.h>
#include <ns3/lte-ue-phy.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/mobility-module.h>
#include <ns3/netanim-module.h>
#include <ns3/network-module.h>
#include <ns3/on-off-helper.h>
#include <ns3/onoff-application.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/psc-module.h>
#include <ns3/simple-device-energy-model.h>
#include <ns3/simulation-callback.h>
#include <ns3/three-gpp-antenna-array-model.h>
#include <ns3/three-gpp-channel-model.h>
#include <ns3/three-gpp-propagation-loss-model.h>
#include <ns3/three-gpp-spectrum-propagation-loss-model.h>
#include <ns3/three-gpp-v2v-propagation-loss-model.h>
#include <random>
#include <sstream>      // std::stringstream
#include <stdio.h>
#include <stdlib.h>     /* abs */
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <typeinfo>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ns3/flow-monitor-module.h"

using namespace ns3;
using namespace psc;

#define print(t, x) NS_LOG_UNCOND("\u001b[3m" << Simulator::Now().GetSeconds() << "s\u001b[1m" << "\t[" << t << "]: \u001b[0m	\u001b[3m\u001b[33m" << x << "\u001b[0m")

bool LOADB_ENABLED = false;


int SIMTIME = 120;
int SIMRUNS = 1;

int EVALVID_ID = 1;


uint32_t NUMBER_UES				= 1;
uint32_t NUMBER_BSS				= 1;
uint32_t NUMBER_UAVS			= 0;
int NUMBER_REQUIRED_UAVS		= 10;

double DRONE_ALTITUDE			= 10;
double DRONE_MAX_SPEED			= 160/3.6;	// m/s
double DRONE_MIN_SPEED			= 30/3.6;	// m/s
double INITIAL_BATT_VOLTAGE		= 22.2;
double INITIAL_ENERGY			= 6.3936*1e6;
double MAX_LONGRANGE_REF		= 500;
double MAX_SHORTRANGE_REF		= 300;
double UAV_DIST_RANGE			= 200;
double ALGOPARAMETERS_DIST		= 1000;
bool HANDOVER_ENABLED			= false;
double TXPOWER_UE				= 23; 
double TXPOWER_BSENB			= 46; 
double TXPOWER_UAV				= 23; 

uint8_t BANDWIDTH_BS_ENB		= 100;
uint8_t BANDWIDTH_UAV_ENB		= 100;

uint8_t BANDWIDTH_BS_ENB_UP		= 25;
uint8_t BANDWIDTH_BS_ENB_DL		= 50;
uint8_t BANDWIDTH_UAV_ENB_UP	= 25;
uint8_t BANDWIDTH_UAV_ENB_DL	= 50;

struct PairdictRsrp{
	int enb_nodeid; // save the id of the node
	double rsrp; // save the sumation of the distances to the 3 suggestions to the alg positions
};
std::map<int, PairdictRsrp> BEST_UE_RSRP;

// std::map<int, double> BEST_UE_SINR;
// std::map<int, int> BEST_UE_ENB;
bool UAV_ASSISTANCE = false;
std::map<int, bool> IS_POORLY_ASSISTED;
std::map<int, double> UE_DATARATE;
std::map<int, double> UE_DATARATE_CURR;
std::map<int, std::map<int, bool>> IS_CONNECTION;
std::map<int, int> UE_ENB_CONNECTION;
std::map<Ipv4Address, int> IPV4ADDRESS_NODEID;
std::map<int, double> UAV_ENERGY_STATUS;

// std::map<int, std::map<int, double>> ENB_UE_BWCAPACITY;

std::map<int, int> PATH_IMSI;
std::map<int, int> CELLID_PATH;
std::map<int, std::map<int, double>> NEIGHBORS_MAP;
std::map<int, std::map<int, int>> RNTI_CELLS;

std::map<int, int> UAV_NUMBERCONNECTIONS;
std::vector<std::vector<int>> CONNECTIONS;

std::map<int, std::string> NODESMAP;

std::vector<int> ENERGY_DEPLETED_DRONES;

std::string NS3_DIR				= "";
std::string TRACEFILE			= "";
std::string PARAMS_PREFIX		= "";
std::string CELLLIST			= "";

NodeContainer UE_NODECONTAINER;
NodeContainer BS_NODECONTAINER;
NodeContainer UAV_NODECONTAINER;
NodeContainer ENB_NODECONTAINER;

NetDeviceContainer UE_LTEDEVS;
NetDeviceContainer UAV_LTEDEVS;
NetDeviceContainer BS_LTEDEVS;
NetDeviceContainer ENB_LTEDEVS;

DeviceEnergyModelContainer UAV_ENERGY_MODE;

FlowMonitorHelper FLOWMON_HELPER;

Ptr<LteHelper> LTE_HELPER;
Ptr<PointToPointEpcHelper> EPC_HELPER;
Ipv4StaticRoutingHelper IPV4ROUTING_HELPER;


std::default_random_engine GENERATOR;
double distance	= 60.0;
Time interPacketInterval = MilliSeconds (200);

std::string	POSITIONALGORITHM		= "random";
int 		RANDOMSEED				= 1000;
bool 		ENABLE_UAVS				= true;
bool 		AUTO_DEPLOYMENT			= true;
bool		USECA					= true;
std::string	RECHARGEPOSITION_STR	= "#0,0,0;";


enum POSITION_ALGORITHM_CODES{
	RANDOM_ALGORITHM	= 101
};

enum APPROACHES_CODES{
	NET_ESTIMATION		= 201,
	RANGE_ESTIMATION	= 202
};

struct SIMULATION_CONFIGURATION{
	POSITION_ALGORITHM_CODES	id_algorithm;
	APPROACHES_CODES			id_approach;
	std::string					name;
	std::string					cmd_str;
} SIM_CONFIG;

struct QoSData{
	double Throughput;
	double PDR;
	double PLR;
	double Delay;
	double Jitter;
};
std::map<int, QoSData> NODES_QOS;

struct PktFlow{
	Time time;
	int pktseq;
	std::string src;
	double size;
};
std::vector< PktFlow > APPLICATION_PACKET_TX;
std::vector< PktFlow > APPLICATION_PACKET_RX;

std::ofstream the_debug_File;
std::string debug_fileName = "Globals-debug.txt";
// void debug(auto content);
std::ostream & trace();

Ptr<ListPositionAllocator> generatePositionAllocator(double area, int number_of_nodes, std::string allocation, double default_altitude);
std::string get_coord_str(Vector location);
// Configure mobility according ga-algorithm
std::string exec(const char* cmd);// Function to execute Python in console
void TurnOffTxPower(int nodeId);
void TurnOnTxPower(int nodeId, double power);
void stop_node(NodeContainer UAV_NODECONTAINER);
void move_node_constVec(Ptr<Node> node, Vector destinationposition, double speed);


std::ostream & trace() {
	if (!the_debug_File.is_open()){
		the_debug_File.open(debug_fileName.c_str(), ios::out);
		the_debug_File.close();
		the_debug_File.clear();
		the_debug_File.open(debug_fileName.c_str(), ios::app);
	}
	the_debug_File.setf(ios::left);

	// return (ostream &) the_debug_File << "\n" << std::setw(4) << Simulator::Now().GetSeconds() << std::setw(8) << " ";
	return (ostream &) the_debug_File << "\n" << Simulator::Now().GetSeconds() << "s,";
}

Ptr<ListPositionAllocator> generatePositionAllocator(
	double area = 10,
	int number_of_nodes = 3,
	std::string allocation = "random",
	double default_altitude = 45
){
	Ptr<ListPositionAllocator> HpnPosition = CreateObject<ListPositionAllocator>();
	// std::uniform_int_distribution<int> distribution(0, area);
	std::uniform_int_distribution<int> distribution(5e3-area, 5e3+area);

	if (allocation == "koln"){
		double multiplier = 1.0 / 10;

		std::ifstream cellList(CELLLIST);

		double a, b, c;
		while (cellList >> a >> b >> c){
			HpnPosition->Add(Vector3D(b * multiplier, c * multiplier, default_altitude));
		}
	} else if (allocation == "random"){
		for (int i = 0; i < number_of_nodes; i++){
			HpnPosition->Add(
				Vector3D(distribution(GENERATOR), distribution(GENERATOR), default_altitude));
		}
	}
	else{
		double multiplier = 1.0;
		std::stringstream file_to_load;
		file_to_load << "./src/GA-drone/my_mobility/" << allocation;

		std::ifstream cellList(file_to_load.str().c_str());
		double a, b, c;
		while (cellList >> a >> b >> c){
			HpnPosition->Add(Vector3D(b * multiplier, c * multiplier, default_altitude));
		}
	}
	return HpnPosition;
}
std::string get_coord_str(Vector location){
	std::string res = "(";
	res += std::to_string(int(location.x)) + ", ";
	res += std::to_string(int(location.y)) + ", ";
	res += std::to_string(int(location.z)) + ") ";
	return res;
}

// Configure mobility according ga-algorithm
std::string exec(const char* cmd)// Function to execute Python in console
{
	std::array<char, 128> buffer;
	std::string result;
	std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
	if (!pipe)
		throw std::runtime_error("popen() failed!");
	while (!feof(pipe.get()))
	{
		if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
			result += buffer.data();
	}
	return result;
}

void TurnOffTxPower(int nodeId){
	for (unsigned int i = 0; i < UAV_LTEDEVS.GetN(); ++i){
		int uav_id = UAV_LTEDEVS.Get(i)->GetNode()->GetId();
		if ((int) uav_id == nodeId){
			Ptr<LteEnbPhy> enb0Phy = UAV_LTEDEVS.Get(i)->GetObject<LteEnbNetDevice>()->GetPhy();

			double txPowerBefore = enb0Phy->GetTxPower();
			enb0Phy->SetTxPower(0.0);
			std::stringstream msg;
			msg << "\u001b[31m["<< __FUNCTION__ << "] " << Simulator::Now().GetSeconds() << "s "
				<< " BEFORE "	<< NODESMAP[uav_id] << "[" << uav_id << "] txPower = " << txPowerBefore 
				<< "\tAFTER "	<< NODESMAP[uav_id] << "[" << uav_id << "] txPower = " << enb0Phy->GetTxPower() << "\u001b[0m";

			// NS_LOG_UNCOND(PARAMS_PREFIX << msg.str().c_str());
			trace() << "[" << __FUNCTION__ << "]," << "Turn OFF tx power of " << NODESMAP[uav_id] << "[" << uav_id << "]";
		}
	}	
}

void TurnOnTxPower(int nodeId, double power){
	for (unsigned int i = 0; i < UAV_LTEDEVS.GetN(); ++i){
		if ((int) UAV_LTEDEVS.Get(i)->GetNode()->GetId() == nodeId){
			int uav_id = UAV_LTEDEVS.Get(i)->GetNode()->GetId();
			Ptr<LteEnbPhy> enb0Phy = UAV_LTEDEVS.Get(i)->GetObject<LteEnbNetDevice>()->GetPhy();

			double txPowerBefore = enb0Phy->GetTxPower();
			enb0Phy->SetTxPower(power);
			std::stringstream msg;
			msg << "\u001b[32m["<< __FUNCTION__ << "] " << Simulator::Now().GetSeconds() << "s "
				<< " BEFORE "	<< NODESMAP[uav_id] << "[" << uav_id << "] txPower = " << txPowerBefore 
				<< "\tAFTER "	<< NODESMAP[uav_id] << "[" << uav_id << "] txPower = " << enb0Phy->GetTxPower() << "\u001b[0m";

			// NS_LOG_UNCOND(PARAMS_PREFIX << msg.str().c_str());
			trace() << "[" << __FUNCTION__ << "]," << "Turn ON tx power of " << NODESMAP[uav_id] << "[" << uav_id << "]";
			
		}
	}	
}

//stop UAV
void stop_node(NodeContainer UAV_NODECONTAINER){
	NS_LOG_UNCOND(PARAMS_PREFIX <<"\u001b[34m["<< __FUNCTION__ <<"] " << Simulator::Now().GetSeconds() << "s " << "\u001b[0m");

	for(unsigned i =0; i < UAV_NODECONTAINER.GetN() ;i++){
		Ptr<ConstantVelocityMobilityModel> mobility = UAV_NODECONTAINER.Get (i)->GetObject<ConstantVelocityMobilityModel>();
		Vector currentPosition = mobility->GetPosition();
		Vector currentSpeed = mobility->GetVelocity();
		mobility->SetPosition(currentPosition);
		mobility->SetVelocity(Vector(0,0,0));
		// NS_LOG_UNCOND("\t(" << Simulator::Now().GetSeconds() << ")\u001b[3m\u001b[35m[" << node->GetId() << "]" << "Arrived to the destination " << get_coord_str(currentPosition) << "\u001b[0m");
		trace() << "[" << __FUNCTION__ << "]," << "[" << UAV_NODECONTAINER.Get (i)->GetId() << "]" << "Arrived to the destination " << get_coord_str(currentPosition) << "\t" << get_coord_str(currentSpeed);
	}
}

void move_node_constVec(Ptr<Node> node, Vector destinationposition, double speed){
	Ptr<ConstantVelocityMobilityModel> mobility = node->GetObject<ConstantVelocityMobilityModel>();
	Vector currentPosition = mobility->GetPosition();
	Vector currentSpeed = mobility->GetVelocity();
	double distance = CalculateDistance(destinationposition, currentPosition);

	// stop_node(node); // Stop at the current position before go to the next destination

	if (currentSpeed.x > 0 ||
		currentSpeed.y > 0 ||
		currentSpeed.z > 0){
		Simulator::Schedule(Seconds(0.0), &stop_node, node); // Stop all UAVs at their current position before go to the next destination
	}
	else{

		std::stringstream log_msg;
		if (distance > 1){

			Vector speedVector = destinationposition - currentPosition;
			speedVector.x = speed*(speedVector.x/distance);
			speedVector.y = speed*(speedVector.y/distance);
			speedVector.z = speed*(speedVector.z/distance);

			mobility->SetVelocity(speedVector);
			
			double currentTime = Simulator::Now().GetSeconds();
			double arrivalTime = (distance/speed) + currentTime;
			
			Simulator::Schedule(Seconds((distance/speed)), &stop_node, node); // the node will stop moving to the direction
			Simulator::Schedule(Seconds((distance/speed)), &TurnOnTxPower, node->GetId(), TXPOWER_UAV); /* Turning on the transmission power of the UAVs after send them to their locations */

			unsigned int nodeId = node->GetId();
			log_msg << "MOVING NODE[" << nodeId << "]"
					<< " FROM: "			<< get_coord_str(currentPosition)			<< ""
					<< " TO: "				<< get_coord_str(destinationposition)		<< ""
					<< " AT SPEED: "		<< get_coord_str(mobility->GetVelocity())	<< "m/s"
					<< " DISTANCE: "		<< distance									<< "m"
					<< " EST.ARV. TIME: "	<< arrivalTime								<< "s"
					<< "";
		}
		else{
			log_msg << " FROM: "			<< get_coord_str(currentPosition)			<< ""
					<< " TO: "				<< get_coord_str(destinationposition)		<< ""
					<< "\tdistance " << distance << "m is too short, not moving";
		}
		NS_LOG_UNCOND(PARAMS_PREFIX <<"\u001b[34m[" << __FUNCTION__ << "] " << Simulator::Now().GetSeconds() << "s " << "\u001b[0m" << log_msg.str().c_str());
		trace() << "[" << __FUNCTION__ << "]," << log_msg.str().c_str();
	}

}




std::vector< ns3::Vector3D > vector_from_string(std::string output){
	Vector a = Vector3D(0,0,0);
	std::vector< ns3::Vector > res;
	std::vector<std::string> splited_coords;
	std::vector<std::string> single_coord;

	if (output.length() < 1)
		return res;

	boost::split(splited_coords, output, boost::is_any_of(";"), boost::token_compress_on);
	for(auto i: splited_coords){
		boost::split(single_coord, i, boost::is_any_of(","), boost::token_compress_on);
		// NS_LOG_UNCOND("vector_from_string: [" << single_coord << "]");
		a.x = std::max(0.0,std::stod(single_coord[0]));
		a.y = std::max(0.0,std::stod(single_coord[1]));
		a.z = abs(std::max(1.5,std::stod(single_coord[2])));
		res.push_back(a);
	}

	return res;
}






void LandDroneToGround(int nodeId){
	print(__FUNCTION__, Simulator::Now().GetSeconds() << "s ");

	for (uint32_t i = 0; i < UAV_NODECONTAINER.GetN(); ++i){
		if ((int) UAV_NODECONTAINER.Get(i)->GetId() == nodeId){
			Ptr<ConstantVelocityMobilityModel> mobility = UAV_NODECONTAINER.Get(i)->GetObject<ConstantVelocityMobilityModel>();
			Vector currentPosition = mobility->GetPosition();
			move_node_constVec(UAV_NODECONTAINER.Get(i), Vector(currentPosition.x, currentPosition.y, 0.001), DRONE_MIN_SPEED);
		}
	}
}


void EnergyDepletedTrigger(int nodeId){
	TurnOffTxPower(nodeId);
	LandDroneToGround(nodeId);
}



int get_path_id(std::string path){
	std::vector<std::string> splited_path;
	boost::split(splited_path, path, boost::is_any_of ("/"), boost::token_compress_on);
	return std::stoi(splited_path[2]);
}


// callback energy depleted
void EnergyDepleted (std::string path, Ptr<const UavMobilityEnergyModel> energyModel){
	int nodeId = get_path_id(path);
	UAV_ENERGY_STATUS[nodeId] = 0.0;


	if (!(std::find(ENERGY_DEPLETED_DRONES.begin(), ENERGY_DEPLETED_DRONES.end(), nodeId) != ENERGY_DEPLETED_DRONES.end())){
		NS_LOG_UNCOND(PARAMS_PREFIX <<"\u001b[34m["<< __FUNCTION__ <<"] " << Simulator::Now().GetSeconds() << "s " << "\u001b[0m"
			<< "ENERGY DEPLETED node [" << nodeId << "]");
		ENERGY_DEPLETED_DRONES.push_back(nodeId);
		Simulator::Schedule(Seconds(0), &EnergyDepletedTrigger, nodeId);
	}

}


void ShowStatus(int deltaTime){
	// NS_LOG_UNCOND(PARAMS_PREFIX <<"\u001b[34m["<< __FUNCTION__ <<"] " << Simulator::Now().GetSeconds() << "s " << "\u001b[0m");

	std::stringstream energyStatus_stream;

	for (unsigned int i = 0; i < UAV_NODECONTAINER.GetN(); ++i){
		int nodeId = UAV_NODECONTAINER.Get(i)->GetId();

		Ptr<LiIonEnergySource> energy_source = UAV_NODECONTAINER.Get(i)->GetObject<LiIonEnergySource>();
		double remainingEnergy = energy_source->GetRemainingEnergy();
		UAV_ENERGY_STATUS[nodeId] = remainingEnergy;
		
		std::ofstream remEnergy;

		std::string filename = "remEnergy.csv";
		remEnergy.open(filename, std::ios::app);

		if(ns3::file_exist(filename))
			if(ns3::is_empty(filename))
				remEnergy << "time," << "nodeId," << "remainingEnergy" << "\n";

		remEnergy << Simulator::Now ().GetSeconds () << "," << nodeId <<  "," << remainingEnergy <<"\n";
		energyStatus_stream << " [" << nodeId << remainingEnergy << "J]";
		remEnergy.close();
	}

	if(UAV_NODECONTAINER.GetN() > 0){
		trace() << "[" << __FUNCTION__ << "]," << "Updating UAVs energy status" << energyStatus_stream.str().c_str();
	}


	Simulator::Schedule(Seconds(deltaTime), &ShowStatus, deltaTime);
}







std::vector< int > getBestNodesSorted(std::vector< Vector > *coords, NodeContainer nodes){

	// ============ Init values ============
	std::vector< Vector > coords_from_alg = *coords;
	std::vector< Vector > coords_from_nod;

	for (uint32_t i = 0; i < nodes.GetN(); ++i){
		int currIndex = i;
		Vector posUAV = nodes.Get(currIndex)->GetObject<MobilityModel>()->GetPosition();
		coords_from_nod.push_back(posUAV);
	}
	// =====================================





	// =====================================
	// Choosing the 3 closest given positions for each drone on the simulation
	int search_suggestions = 3;

	struct pair{
		double distance;
		int index_nod;
		int index_alg;
		Vector3D pos_nod;
		Vector3D pos_alg;
	};

	std::vector<pair> list_pairs;
	for (auto it_nod = coords_from_nod.begin(); it_nod != coords_from_nod.end(); ++it_nod){
		int idx_nod = it_nod - coords_from_nod.begin();

		std::vector< int > list_choosen_indexes_alg;
		for (int count = 0; count < search_suggestions; ++count){

			double shortest_distance = -1;
			int best_index_alg = -1;
			
			for (auto it_alg = coords_from_alg.begin(); it_alg != coords_from_alg.end(); ++it_alg){// find the closest three points from A to B
				int idx_alg = it_alg - coords_from_alg.begin();
				
				double current_distance = CalculateDistance(*it_nod, *it_alg);
				if ((current_distance < shortest_distance || shortest_distance < 0) &&
					!(std::find(list_choosen_indexes_alg.begin(), list_choosen_indexes_alg.end(), idx_alg) != list_choosen_indexes_alg.end())){
					shortest_distance = current_distance;
					best_index_alg = idx_alg;
				}
			}

			if (shortest_distance >= 0 && best_index_alg >= 0){
				list_choosen_indexes_alg.push_back(best_index_alg);
				pair newPair;
				newPair.distance = shortest_distance;
				newPair.index_nod = idx_nod;
				newPair.index_alg = best_index_alg;
				list_pairs.push_back(newPair);
			}
		}
	}
	// =====================================





	// =====================================
	// ===== Sort the nodes according to thefarthest distance from the alg positions =====
	struct id_sum_distance{
		int id; // save the id of the node
		double sum_distance; // save the sumation of the distances to the 3 suggestions to the alg positions
	};
	auto compareSumDistance = [](const id_sum_distance &a, const id_sum_distance &b) {
		return a.sum_distance > b.sum_distance;
	};



	double distance_sum = 0.0;
	if (list_pairs.size() > 0){
		int current_index_nod = list_pairs.begin()->index_nod;
		std::vector< id_sum_distance > distace_sum_list_nod; // list of nod and their distance sumation which will be sorted

		for (auto it_pair = list_pairs.begin(); it_pair != list_pairs.end(); it_pair++ ){
			
			if (it_pair->index_nod != current_index_nod){
				// NS_LOG_UNCOND("nod[" << current_index_nod << "]" << " distance_sum: " << distance_sum);
				// NS_LOG_UNCOND(" ");
				id_sum_distance newInput;
				newInput.id = current_index_nod;
				newInput.sum_distance = distance_sum;
				distace_sum_list_nod.push_back(newInput);

				distance_sum = 0.0; // reseting the sumation for the next nod
				current_index_nod = it_pair->index_nod;
			}
			if (it_pair->index_nod == current_index_nod){
				distance_sum += it_pair->distance;
			}

		}
		id_sum_distance newLastInput;
		newLastInput.id = current_index_nod;
		newLastInput.sum_distance = distance_sum;
		distace_sum_list_nod.push_back(newLastInput);


		std::sort(distace_sum_list_nod.begin(), distace_sum_list_nod.end(), compareSumDistance);
	
	// =====================================



		list_pairs.erase(list_pairs.begin(), list_pairs.end());

		std::vector< int > list_choosen_indexes_alg;
		for (auto it_sum_list = distace_sum_list_nod.begin(); it_sum_list != distace_sum_list_nod.end(); ++it_sum_list){

			double shortest_distance = -1;
			int best_index_alg = -1;
			
			for (auto it_alg = coords_from_alg.begin(); it_alg != coords_from_alg.end(); ++it_alg){// find the closest three points from A to B
				int idx_alg = it_alg - coords_from_alg.begin();
				
				double current_distance = CalculateDistance(coords_from_nod[it_sum_list->id], *it_alg);

				if ((current_distance < shortest_distance || shortest_distance < 0) &&
					!(std::find(list_choosen_indexes_alg.begin(), list_choosen_indexes_alg.end(), idx_alg) != list_choosen_indexes_alg.end())){
					shortest_distance = current_distance;
					best_index_alg = idx_alg;
				}
			}

			if (shortest_distance >= 0 && best_index_alg >= 0){
				list_choosen_indexes_alg.push_back(best_index_alg);
				pair newPair;
				newPair.distance = shortest_distance;
				newPair.index_nod = it_sum_list->id;
				newPair.index_alg = best_index_alg;
				list_pairs.push_back(newPair);
			}
			else{
				pair newPair;
				newPair.distance = shortest_distance;
				newPair.index_nod = it_sum_list->id;
				newPair.index_alg = best_index_alg;
				list_pairs.push_back(newPair);
			}

		}
	}

	std::vector< int > sorted_index_nod;
	std::vector< Vector > sorted_coords_alg;

	for (auto it_pair = list_pairs.begin(); it_pair != list_pairs.end(); it_pair++ ){
		if (it_pair->index_nod > -1){
			sorted_index_nod.push_back(it_pair->index_nod);
		}
		if (it_pair->index_alg > -1){
			sorted_coords_alg.push_back(coords_from_alg[it_pair->index_alg]);
		}
	}

	*coords = sorted_coords_alg;

	return sorted_index_nod;
}

NodeContainer getPoorlyAssistendNodes(){
	NodeContainer res;
	for(unsigned i = 0; i < UE_NODECONTAINER.GetN(); i++){
		int nodeid = UE_NODECONTAINER.Get(i)->GetId();
		if(IS_POORLY_ASSISTED[nodeid]){
			res.Add(UE_NODECONTAINER.Get(i));
		}
	}
	if(res.GetN() <= 0 && Simulator::Now().GetSeconds() > 5){
		Simulator::Stop();
	}
	return res;
}

void TriggerUavAssistance(){
	UAV_ASSISTANCE = true;
}
void UnTriggerUavAssistance(){
	UAV_ASSISTANCE = false;
}

void updateDronePosition(std::string (*f)(std::string pos, int maxUAVs), Time deltaTime){
	std::string targetPosStr = "";
	NodeContainer poorlyAssistedNodes = getPoorlyAssistendNodes();
	
	/* Select the UEs that need to be attended */
	for (uint32_t i = 0; i < poorlyAssistedNodes.GetN(); ++i){
		Ptr<Node> ue_node = poorlyAssistedNodes.Get(i);
		Vector3D pos = ue_node->GetObject<MobilityModel>()->GetPosition();
		targetPosStr += std::to_string(pos.x)+",";
		targetPosStr += std::to_string(pos.y)+",";
		targetPosStr += std::to_string(pos.z)+";";
	}




	if (targetPosStr.length() < 1){
		print(__FUNCTION__, Simulator::Now().GetSeconds() << "s " << "skipping update...");
		return;
	}










	if (deltaTime.GetSeconds() > 0 && Simulator::Now().GetSeconds() > 1){
		bool areSomeOfThemStillMoving = false;
		for(unsigned i = 0; i < UAV_NODECONTAINER.GetN(); i++){
			Ptr<ConstantVelocityMobilityModel> mobility = UAV_NODECONTAINER.Get(i)->GetObject<ConstantVelocityMobilityModel>();
			Vector speed = mobility->GetVelocity();
			if( abs(speed.x) > 0 || abs(speed.y) > 0 || abs(speed.z) > 0 ){
				print(__FUNCTION__, "speed.x: " << speed.x << "\tspeed.y: " << speed.y << "\tspeed.z: " << speed.z << "\t still moving");
				areSomeOfThemStillMoving = true;
				return;
				break;
			}
		}



		
		if(!areSomeOfThemStillMoving){
			// print(__FUNCTION__, Simulator::Now().GetSeconds() << "s " << "updating positions... " << " targetPosStr: " << targetPosStr);

			std::string newPos_str = (*f)(targetPosStr, UAV_NODECONTAINER.GetN()); //std::string newPos_str = (*f)(targetPosStr, NODETYPE_MAP_LIST["drone"].size());
			if (newPos_str.length() > 1){
				// print(__FUNCTION__, "newPos_str: " << newPos_str);

				std::string elapsed_time = newPos_str.substr(0, newPos_str.find("#"));
				// print(__FUNCTION__, "elapsed_time: " << elapsed_time);

				newPos_str = newPos_str.substr(newPos_str.find("#") + 1);
				newPos_str.pop_back();
				std::vector< Vector > newPos_vector = vector_from_string(newPos_str);


				std::vector< int > nodeIDs = getBestNodesSorted(&newPos_vector, UAV_NODECONTAINER);
				for (uint32_t i = 0; i < UAV_NODECONTAINER.GetN(); ++i){
					if (i >= nodeIDs.size() || i >= newPos_vector.size()){ // prevent vector access error
						break;
					}
					Simulator::Schedule(Seconds (0.0), &move_node_constVec, UAV_NODECONTAINER.Get(nodeIDs[i]), newPos_vector[i], DRONE_MAX_SPEED);
				}
			}
		}
	}
}






std::string load_random_positions_str(std::string targetPosStr, int maxUAVs){
	// Loading cover positions from external algorithm
	std::stringstream cmd;
	cmd << "python3 python_scripts/get_random_positions.py "
		<< " --pos \"" << targetPosStr << "\""
		<< " -a " << DRONE_ALTITUDE << " "
		<< " -k " << maxUAVs << " "
		<< " 2>/dev/null ";

	std::string output = exec(cmd.str().c_str());
	if(output.length() < 1){
		output = targetPosStr;
	}

	print(__FUNCTION__, "cmd: " << cmd.str().c_str());
	print(__FUNCTION__, "output: " << output);

	return output;
}


Ptr<ListPositionAllocator> load_random_positions(NodeContainer targetNodes, double altitude, uint32_t *numberCoverNodes){
	// Loading user initial UE locations
	std::string targetPosStr = "";
	for (uint32_t i = 0; i < targetNodes.GetN(); ++i){
		Vector pos = targetNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
		targetPosStr += std::to_string(pos.x)+",";
		targetPosStr += std::to_string(pos.y)+",";
		targetPosStr += std::to_string(pos.z)+";";
	}

	// Loading cover positions from external algorithm
	std::stringstream cmd;
	cmd << "python3 python_scripts/get_random_positions.py "
		<< " --pos \"" << targetPosStr << "\""
		<< " -a " << DRONE_ALTITUDE << " "
		<< " -k " << NUMBER_UAVS << " "
		<< " 2>/dev/null ";
	SIM_CONFIG.cmd_str = cmd.str().c_str();

	print(__FUNCTION__, get_current_dir_name());
	std::string output = "";
	output = exec(cmd.str().c_str());
	// output = "1.0#0.008897066116333008#1695.829397710542,1433.534709652705,10.0;3315.0749253579033,1876.887907020544,10.0;3009.878823770514,3708.1452859870196,10.0;2334.8538974799894,3801.1534074154592,10.0;";

	print(__FUNCTION__, "cmd: " << cmd.str().c_str());
	print(__FUNCTION__, "output: " << output);

	std::string elapsed_time = output.substr(0, output.find("#"));
	output = output.substr(output.find("#") + 1);
	output.pop_back();

	std::vector< Vector > newPos_vector;
	try
	{
		newPos_vector = vector_from_string(output);
		*numberCoverNodes = newPos_vector.size();
		// NS_LOG_UNCOND("\u001b[33mNumber of UAVs: \u001b[1m" << *numberCoverNodes << "\u001b[0m");
		// Loading into simulation format
	}
	catch(const std::exception& e)
	{
		print(__FUNCTION__, "\u001b[31m Error running \u001b[3m" << cmd.str().c_str() << " \u001b[0m\u001b[3m\u001b[33m");
	}
	Ptr<ListPositionAllocator> positionAllocator = CreateObject<ListPositionAllocator> ();
	for (int i = 0; i < (int) newPos_vector.size(); ++i){
		positionAllocator->Add (Vector( newPos_vector[i].x, newPos_vector[i].y, newPos_vector[i].z));
	}
	
	return positionAllocator;
}








void show_connections(){
	print(__FUNCTION__, "[" << __FUNCTION__ << "]");
	for (auto const& x : IS_CONNECTION){
		for (auto const& y : x.second){
			print(__FUNCTION__, "IS_CONNECTION[UE " << x.first << "][eNB " << y.first << "] = " << y.second);
		}
	}
}

void show_neighbors(){
	print(__FUNCTION__, "[" << __FUNCTION__ << "]");
	for (auto const& x : NEIGHBORS_MAP){
		for (auto const& y : x.second){
			print(__FUNCTION__, "NEIGHBORS_MAP [eNB " << x.first << "][UE " << y.first << "]\t= " << y.second);
		}
	}
}


// populate pairing from nodeId to imsi
int populate_path_imsi(std::string path, int imsi){
	int nodeId = get_path_id(path);

	if (imsi != -1){
		PATH_IMSI[nodeId] = imsi;
	}

	return nodeId;
}


int get_imsi(int cellid, int rnti){
	return RNTI_CELLS[cellid][rnti] == 0 ? -1 : RNTI_CELLS[cellid][rnti];
}


int getServingcell(int imsi){
	int servingCell = 0;
	
	for (uint32_t i = 0; i < ENB_NODECONTAINER.GetN(); i++)
		if (CONNECTIONS[i][imsi - 1] != 0)
			servingCell = i;
	
	return servingCell;
}

int getServingcell_map(int nodeId){
	// RESETTING THE CONNECTIONS
	map<int, bool>::iterator it;
	for (it = IS_CONNECTION[nodeId].begin (); it != IS_CONNECTION[nodeId].end (); it++){
		if(IS_CONNECTION[nodeId][it->first]){
			return it->first;
		}
	}
	
	return -1;
}


Ptr<Node> get_node_index(NodeContainer n, uint32_t nodeid){
	Ptr<Node> res;
	for(unsigned i = 0; i < n.GetN(); i++){
		if(n.Get(i)->GetId() == nodeid){
			res = n.Get(i);
			break;
		}
	}
	return res;
}

Ptr<NetDevice> get_node_index(NetDeviceContainer n, uint32_t nodeid){
	Ptr<NetDevice> res;
	for(unsigned i = 0; i < n.GetN(); i++){
		if(n.Get(i)->GetNode()->GetId() == nodeid){
			res = n.Get(i);
			break;
		}
	}
	return res;
}




int get_nodeid_from_path(std::string path){
	int nodeid;

	std::vector<std::string> split_path;
	boost::split(split_path, path, boost::is_any_of("/"));
	nodeid = stoi(split_path[2]);

	// if key is present, return nodeid which is 1 less than imsi
	if (PATH_IMSI.find(nodeid) != PATH_IMSI.end()){
		return PATH_IMSI[nodeid] - 1;
	}
	return -1;
}







// void RecvMeasurementReportCallback(std::string path, uint64_t imsi, uint16_t cellid, uint16_t rnti, LteRrcSap::MeasurementReport meas){
// }

// void ReportCurrentCellRsrpSinr(std::string path, uint16_t cellid, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId){
// }





void ReportUeMeasurements(std::string path,
	uint16_t rnti,		uint16_t cellid,
	double rsrp,		double rsrq,
	bool servingCell,	uint8_t componentCarrierId 
){
	print(__FUNCTION__, "report");
}

void NotifyHandoverStartEnb(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId){
	print(__FUNCTION__, "Handover start enb");
}

void NotifyHandoverStartUe(std::string path, uint64_t imsi, uint16_t cellid, uint16_t rnti, uint16_t targetcellid){
	print(__FUNCTION__, "Handover start ue");

}
void NotifyHandoverEndOkUe(std::string path, uint64_t imsi, uint16_t cellid, uint16_t rnti){
	print(__FUNCTION__, "Handover end ok");
}




void NotifyConnectionEstablishedEnb (std::string path, uint64_t imsi, uint16_t cellid, uint16_t rnti){
	int enb_id = get_path_id(path);
	print(__FUNCTION__, "New connection: eNB[" << enb_id << "] connected to cellid[" << cellid << "] rnti[" << rnti << "]");
}


void NotifyConnectionEstablishedUe (std::string path, uint64_t imsi, uint16_t cellid, uint16_t rnti){
	print(__FUNCTION__, "");
}

void ReportInterference(std::string context, unsigned short cellId, Ptr<SpectrumValue> interference){
	print(__FUNCTION__, "");
}

void RrcTimeoutEnb(std::string context, unsigned long val1, unsigned short val2, unsigned short val3, std::string val4){
	int enb_id = get_path_id(context);
	print(__FUNCTION__, Simulator::Now().GetSeconds() << "\teNB["<< enb_id <<"]");
}

void InitialCellSelectionEndError(std::string context, uint64_t imsi, uint16_t cellId){
	int ue_id = get_path_id(context);
	print(__FUNCTION__, "UE["<< ue_id <<"]");
}
void RandomAccessError(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti){
	int ue_id = get_path_id(context);
	print(__FUNCTION__, "UE["<< ue_id <<"]");
}
void ConnectionTimeout(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint8_t connEstFailCount){
	int ue_id = get_path_id(context);
	print(__FUNCTION__, "UE["<< ue_id <<"]");
}
void HandoverEndError(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti){
	int ue_id = get_path_id(context);
	print(__FUNCTION__, "UE["<< ue_id <<"]");
}
void UEConnectionReconfiguration(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti){
	int ue_id = get_path_id(context);
	print(__FUNCTION__, "UE["<< ue_id <<"]");
}


void NotifyRaResponseTimeoutUe(std::string path, uint64_t imsi, bool contention, uint8_t preambleTxCounter, uint8_t maxPreambleTxLimit){
	int ue_id = get_path_id(path);
	print(__FUNCTION__, "UE["<< ue_id <<"]\tcontention [" << contention << "]\t" << "preambleTxCounter["<< preambleTxCounter <<"]\tmaxPreambleTxLimit["<< maxPreambleTxLimit <<"]");
}

void LteEnbDlRxError(std::string context, Ptr<const Packet> packet){
	int enb_id = get_path_id(context);
	print(__FUNCTION__, "eNB["<< enb_id <<"] drop packet [" << packet->GetUid() << "]");
}
void LteEnbUlRxError(std::string context, Ptr<const Packet> packet){
	int enb_id = get_path_id(context);
	print(__FUNCTION__, "eNB["<< enb_id <<"] drop packet [" << packet->GetUid() << "]");
}
void LteUeDlRxError(std::string context, Ptr<const Packet> packet){
	int ue_id = get_path_id(context);
	print(__FUNCTION__, "UE["<< ue_id <<"] drop packet [" << packet->GetUid() << "]");
}
void LteUeUlRxError(std::string context, Ptr<const Packet> packet){
	int ue_id = get_path_id(context);
	print(__FUNCTION__, "UE["<< ue_id <<"] drop packet [" << packet->GetUid() << "]");
}
void MacTxDrop(std::string context, Ptr<const Packet> packet){
	int nodeid = get_path_id(context);
	std::string nodetype = NODESMAP[nodeid];
	print(__FUNCTION__, nodetype <<" ["<< nodeid <<"] drop packet [" << packet->GetUid() << "]");
}
void MacRxDrop(std::string context, Ptr<const Packet> packet){
	int nodeid = get_path_id(context);
	std::string nodetype = NODESMAP[nodeid];
	print(__FUNCTION__, nodetype <<" ["<< nodeid <<"] drop packet [" << packet->GetUid() << "]");
}
void PhyRxDrop(std::string context, Ptr<const Packet> packet){
	int nodeid = get_path_id(context);
	std::string nodetype = NODESMAP[nodeid];
	print(__FUNCTION__, nodetype <<" ["<< nodeid <<"] drop packet [" << packet->GetUid() << "]");
}
void PhyTxDrop(std::string context, Ptr<const Packet> packet){
	int nodeid = get_path_id(context);
	std::string nodetype = NODESMAP[nodeid];
	print(__FUNCTION__, nodetype <<" ["<< nodeid <<"] drop packet [" << packet->GetUid() << "]");
}


void UpdatePathloss(std::string path, Ptr<const SpectrumPhy> txPhyConst, Ptr<const SpectrumPhy> rxPhyConst, double lossDb){
	print(__FUNCTION__, path << " " << lossDb);
}

void QoSMonitor(Ptr<FlowMonitor> flowMon, double deltaTime){
	Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (FLOWMON_HELPER.GetClassifier ());
	flowMon->CheckForLostPackets ();
	classing = DynamicCast<Ipv4FlowClassifier> (FLOWMON_HELPER.GetClassifier ());
	FlowMonitor::FlowStatsContainer stats = flowMon->GetFlowStats ();
	auto flowStats = flowMon->GetFlowStats();

	
	double TxPacket_sum		= 0.0;
	double TxBytes_sum		= 0.0;
	double RxPacket_sum		= 0.0;
	double RxBytes_sum		= 0.0;
	double LostPacket_sum	= 0.0;
	double Delay_sum		= 0.0;
	double Throughput_sum	= 0.0;
	double Jitter_sum		= 0.0;

	int flowCounter = 0;

	int countPoorlyAssisted = 0;
	for (auto stats : flowStats){
		double PDR			= 0.0;
		double PLR			= 0.0;
		double Delay		= 0.0;
		double Throughput	= 0.0;
		double Jitter		= 0.0;
		
		Ipv4FlowClassifier::FiveTuple t = classing->FindFlow(stats.first);
		std::map<Ipv4Address, int>::const_iterator it_sourceAddress = IPV4ADDRESS_NODEID.find( t.sourceAddress );
		std::map<Ipv4Address, int>::const_iterator it_destinationAddress = IPV4ADDRESS_NODEID.find( t.destinationAddress );

		if(it_sourceAddress != IPV4ADDRESS_NODEID.end() && it_destinationAddress != IPV4ADDRESS_NODEID.end()){
			int sourceNode = IPV4ADDRESS_NODEID[t.sourceAddress];
			int destinationNode = IPV4ADDRESS_NODEID[t.destinationAddress];
			double LostPacket = stats.second.txPackets - stats.second.rxPackets;

			PDR = (double) stats.second.rxPackets / (double) stats.second.txPackets;
			PLR = (double) LostPacket / (double) stats.second.txPackets;
			Delay = (stats.second.delaySum.GetMilliSeconds()) / (stats.second.txPackets);
			Throughput = (double) stats.second.rxBytes * 8.0 /
				(double) (stats.second.timeLastRxPacket.GetSeconds() - stats.second.timeFirstTxPacket.GetSeconds());
			// Jitter = 1024 * stats.second.jitterSum.GetSeconds () / stats.second.rxPackets;
			Jitter = stats.second.jitterSum.GetMilliSeconds() / (double) stats.second.rxPackets;

			QoSData newQoS;
			newQoS.Throughput = Throughput;
			newQoS.PDR = PDR;
			newQoS.PLR = PLR;
			newQoS.Delay = Delay;
			newQoS.Jitter = Jitter;
			NODES_QOS[sourceNode] = newQoS;
			
			// LOADING NEIGHBORS ID TO SHOW IN THE LOGS:
			std::stringstream neighbors_list;
			for (auto const& x : NEIGHBORS_MAP){
				for (auto const& y : x.second){
					// print(__FUNCTION__, "NEIGHBORS_MAP [eNB " << x.first << "][UE " << y.first << "]\t= " << y.second);
					if(y.first == sourceNode && y.second > -DBL_MAX){
						neighbors_list << "[" << NODESMAP[x.first] << " " << x.first << " " << (int) y.second << "] ";
					}
					if(y.first == sourceNode && y.second <= -DBL_MAX){
						neighbors_list << "[-] ";
					}
				}
			}
			
			
			
			double lenience = 0.8;
			IS_POORLY_ASSISTED[sourceNode] = (Throughput < (lenience * UE_DATARATE[sourceNode]));
			UE_DATARATE_CURR[sourceNode] = UE_DATARATE[sourceNode]-Throughput;


			if(IS_POORLY_ASSISTED[sourceNode]){
				countPoorlyAssisted += 1;
				print(__FUNCTION__, ""
					<< "SRC: "<< NODESMAP[sourceNode] <<"["<< sourceNode <<"] " << t.sourceAddress << "\t"
					<< "DEST: "<< NODESMAP[destinationNode] <<"["<< destinationNode <<"] " << t.destinationAddress << "\t"
					<< "eNB: "<< NODESMAP[UE_ENB_CONNECTION[sourceNode]] <<"["<< UE_ENB_CONNECTION[sourceNode] <<"]\t"
					<< "\u001b[3m\u001b[31m\u001b[1m"
					<< "Throughput ["<< min(UE_DATARATE[sourceNode], Throughput)/1e6 <<" Mbps] of ["<< UE_DATARATE[sourceNode]/1e6 <<" Mbps]\t"
					<< "");
			}

			trace() << "[" << __FUNCTION__ << "],"
				<< "timeLastTxPacket: " << stats.second.timeLastTxPacket.GetMilliSeconds() << "ms "
				<< "Pkt flow: " << NODESMAP[sourceNode] <<"["<< sourceNode <<"](" << t.sourceAddress << ")->"
				<< "eNB_"<< NODESMAP[UE_ENB_CONNECTION[sourceNode]] <<"["<< UE_ENB_CONNECTION[sourceNode] <<"]->"
				<< NODESMAP[destinationNode] <<"["<< destinationNode <<"](" << t.destinationAddress << ") ";
			trace() << "[" << __FUNCTION__ << "],"
				<< "Flow status: "
				<< "PDR[" << std::setprecision(3) << (PDR*100) <<" %],"
				<< "PLR[" << std::setprecision(3) << (PLR*100) <<" %],"
				<< "Delay[" << std::setprecision(3) << Delay <<" ms],"
				<< "Jitter[" << std::setprecision(3) << max(0.0, Jitter) <<" ms],"
				<< "Throughput ["	<< std::setprecision(3) << min((UE_DATARATE[sourceNode]/1e6), (double) (Throughput)/1e6)
						  << " of " << std::setprecision(3) << UE_DATARATE[sourceNode]/1e6 <<" Mbps],"
				<< "";

			if ((stats.second.timeLastRxPacket.GetSeconds() - stats.second.timeFirstTxPacket.GetSeconds()) > 0){
				LostPacket_sum += std::max(0.0, (double) LostPacket);
				TxPacket_sum += std::max(0.0, (double) stats.second.txPackets);
				TxBytes_sum += std::max(0.0, (double) stats.second.txBytes);
				RxPacket_sum += std::max(0.0, (double) stats.second.rxPackets);
				RxBytes_sum += std::max(0.0, (double) stats.second.rxBytes);
				Delay_sum += std::max(0.0, (double) Delay);
				Throughput_sum += std::max(0.0, min(UE_DATARATE[sourceNode], (double) Throughput));
				Jitter_sum += std::max(0.0, (double) Jitter);
				flowCounter++;
			}
		}
	}

	double TxPackets_avg = TxPacket_sum / flowCounter;
	double TxBytes_avg = TxBytes_sum / flowCounter;
	double RxPackets_avg = RxPacket_sum / flowCounter;
	double RxBytes_avg = RxBytes_sum / flowCounter;
	double PLR_avg = LostPacket_sum / TxPacket_sum;
	double PDR_avg = RxPacket_sum / TxPacket_sum;
	double Delay_avg = Delay_sum / flowCounter;
	double Throughput_avg = Throughput_sum / flowCounter;
	double Jitter_avg = Jitter_sum / flowCounter; // MilliSeconds

	int totalConnected = 0; // std::stringstream log_msg;
	// IS_CONNECTION[ue_id][enb_id]
	for (auto const& ue : IS_CONNECTION){
		for (auto const& enb : ue.second){
			if (enb.second){
				totalConnected++;
			}
		}
	}

	// print(log_msg.str().c_str());
	double remainingEnergySum = 0;
	int nodes_alive = 0;
	for (uint32_t i = 0; i < UAV_NODECONTAINER.GetN(); ++i){
		// int nodeId = UAV_NODECONTAINER.Get(i)->GetObject<Node>()->GetId();
		Ptr<LiIonEnergySource> energy_source = UAV_NODECONTAINER.Get(i)->GetObject<LiIonEnergySource>();
		double remainingEnergy = energy_source->GetRemainingEnergy();
		// // print("\t INITIAL_ENERGY = " << INITIAL_ENERGY << "\t\tenergy_source->GetInitialEnergy = " << energy_source->GetInitialEnergy()  << "\t\tremainingEnergy = " << remainingEnergy);
		// if (remainingEnergy > INITIAL_ENERGY){
		// 	NS_ASSERT_MSG(false, "\u001b[31m nodeId[" << nodeId << "] INITIAL_ENERGY < remainingEnergy: " << INITIAL_ENERGY << " < " << remainingEnergy << ": \u001b[0m");
		// }
		if (remainingEnergy <= 10){
			// print("\t nodeId[" << nodeId << "] remainingEnergy <= 0: " << remainingEnergy);
			// NS_ASSERT_MSG(false, "\u001b[31m nodeId[" << nodeId << "]: " << remainingEnergy << " <= 0: \u001b[0m");
			remainingEnergy = 0;
		}
		else{
			nodes_alive++;
		}
		remainingEnergySum += remainingEnergy;
	}










	std::string header = "";
	header.append("N_uavs,");
	header.append("N_ues,");
	header.append("N_enbs,");
	header.append("proposal,");
	header.append("PDR,");
	header.append("PLR,");
	header.append("delay,");
	header.append("throughput,");
	header.append("avg_energy_uavs,");
	header.append("jitter,");
	header.append("TxPackets,");
	header.append("TxBytes,");
	header.append("RxPackets,");
	header.append("RxBytes,");

	std::stringstream line;
		line << UAV_NODECONTAINER.GetN()									// "N_uavs"
		<< "," << UE_NODECONTAINER.GetN()									// "N_ues"
		<< "," << BS_NODECONTAINER.GetN()									// "N_enbs"
		<< "," << SIM_CONFIG.name											// "proposal"
		<< "," << PDR_avg													// "PDR"
		<< "," << PLR_avg													// "PLR"
		<< "," << Delay_avg													// "delay"
		<< "," << Throughput_avg											// "throughput"
		<< "," << (double)(remainingEnergySum / UAV_NODECONTAINER.GetN())	// "avg_energy_uavs"
		<< "," << Jitter_avg												// "jitter"
		<< "," << TxPackets_avg												// "TxPackets"
		<< "," << TxBytes_avg												// "TxBytes"
		<< "," << RxPackets_avg												// "RxPackets"
		<< "," << RxBytes_avg												// "RxBytes"
		<< "";
	
	std::string fileName = __FUNCTION__;
	ns3::printFile(fileName, header, line.str().c_str(), "");
	header = "proposal,N_ues,N_enbs,N_uavs,Datarate,Throughput,PDR,PLR,Delay,Jitter";

	for(auto const& x: NODES_QOS){
		line.str("");
		line << ""
			<< SIM_CONFIG.name << ","
			<< max(0, (int) UE_NODECONTAINER.GetN()) << ","
			<< max(0, (int) BS_NODECONTAINER.GetN()) << ","
			<< max(0, (int) UAV_NODECONTAINER.GetN()) << ","
			<< UE_DATARATE[x.first] << ","
			<< min(UE_DATARATE[x.first], max(0.0, (double) x.second.Throughput)) << ","
			<< max(0.0, (double) x.second.PDR) << ","
			<< max(0.0, (double) x.second.PLR) << ","
			<< max(0.0, (double) x.second.Delay) << ","
			<< max(0.0, (double) x.second.Jitter);
		
		std::stringstream context;
		context << "././" << x.first;
		ns3::printFile(fileName+"_singlenode", header, line.str().c_str(), context.str().c_str());
	}

	int number_poorly_assisted = 0;
	for(auto const& x: IS_POORLY_ASSISTED)
		if(x.second)
			number_poorly_assisted++;

	if (
		ENABLE_UAVS /* Check if the UAVs are enabled in the simulation */
		&& number_poorly_assisted > 0 /* Check if there are any poorly assisted UE */
	){
		Time updateRate = Seconds(1);
		Simulator::Schedule(updateRate, &updateDronePosition, load_random_positions_str, updateRate);
	}


	
	fileName = "Poorly_assisted";
	header = "absolute,total,rate";
	line.str("");
	line << number_poorly_assisted << ",";
	line << UE_NODECONTAINER.GetN() << ",";
	line << number_poorly_assisted/UE_NODECONTAINER.GetN();

	ns3::printFile(fileName, header, line.str().c_str(), "");

	Simulator::Schedule(Seconds(deltaTime), &QoSMonitor, flowMon, deltaTime);
}

void SaveNodesPosition(Time interval){
	// print(__FUNCTION__, "Saving movement in " << __FUNCTION__ << ".csv");
	NodeContainer allNodes;
	if(UE_NODECONTAINER.GetN() > 0)
		allNodes.Add(UE_NODECONTAINER);
	if(BS_NODECONTAINER.GetN() > 0)
		allNodes.Add(BS_NODECONTAINER);
	if(UAV_NODECONTAINER.GetN() > 0)
		allNodes.Add(UAV_NODECONTAINER);
	
	std::string header;

	
	header = "type,x,y,z,status";
	
	for(unsigned i = 0; i < allNodes.GetN(); i++){
		int nodeid = allNodes.Get(i)->GetId();
		Vector currentPosition;
		currentPosition = allNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
		
		std::stringstream line;
		line << "" << NODESMAP[nodeid] << "," << currentPosition.x << "," << currentPosition.y << "," << currentPosition.z;
		
		if(NODESMAP[nodeid] == "UE" && IS_POORLY_ASSISTED[nodeid]){
			line << ",poorly_assisted";
		}
		else if(NODESMAP[nodeid] == "UAV" && UAV_ENERGY_STATUS[nodeid] == 0){
			line << ",energy_depleted";
		}
		else{
			line << ",ok";
		}

		std::stringstream context;
		context << "././" << nodeid;
		ns3::printFile(__FUNCTION__, header, line.str().c_str(), context.str().c_str());
		
	}
	Simulator::Schedule(interval, &SaveNodesPosition, interval);
}


inline bool dir_exist (const std::string& name) {
	struct stat buffer;   
	return (stat (name.c_str(), &buffer) == 0); 
}



NS_LOG_COMPONENT_DEFINE ("UEs, UAVs and BSs");

int main (int argc, char *argv[]){
	// LogComponentEnable("V4Ping", LOG_LEVEL_ALL);
	// LogComponentEnable("ArpL3Protocol", LOG_LEVEL_ALL);
	// LogComponentEnable("ThreeGppPropagationLossModel", LOG_LEVEL_ALL);
	// LogComponentEnable("LteEnbRrc", LOG_LEVEL_ALL);

	CommandLine cmd;
	cmd.AddValue("simTime",				"Simulation time",							SIMTIME);
	cmd.AddValue("simRuns",				"Number of runs",							SIMRUNS);
	cmd.AddValue("n_ue",				"Number of UEs",							NUMBER_UES);
	cmd.AddValue("n_enb",				"Number of eNBs",							NUMBER_BSS);
	cmd.AddValue("n_uav",				"Number of UAVs",							NUMBER_UAVS);
	cmd.AddValue("bs_txpower",			"bs_txpower",								TXPOWER_BSENB);
	cmd.AddValue("uav_txpower",			"uav_txpower",								TXPOWER_UAV);
	cmd.AddValue("ue_txpower",			"ue_txpower",								TXPOWER_UE);
	cmd.AddValue("maxLongRange",		"Long communication range reference",		MAX_LONGRANGE_REF);
	cmd.AddValue("maxShortRange",		"Short communication range reference",		MAX_SHORTRANGE_REF);
	cmd.AddValue("uavDistRange",		"Distance among UAVs",						UAV_DIST_RANGE);
	cmd.AddValue("initialEnergy",		"initialEnergy",							INITIAL_ENERGY);
	cmd.AddValue("batteryVoltage",		"batteryVoltage",							INITIAL_BATT_VOLTAGE);
	cmd.AddValue("randomSeed",			"value of seed for random",					RANDOMSEED);
	cmd.AddValue("enable_uavs",			"Enable UAVs",								ENABLE_UAVS);
	cmd.AddValue("auto_deployment",		"Enable auto deployment",					AUTO_DEPLOYMENT);
	cmd.Parse (argc, argv);

	// NUMBER_BSS = max(1,min(247, (int) NUMBER_BSS));
	NUMBER_BSS = max(0,min(247, (int) NUMBER_BSS));
	NUMBER_UES = max(1,min(300, (int) NUMBER_UES));

	print(__FUNCTION__, "SIMTIME \t\t= "				<< SIMTIME);
	print(__FUNCTION__, "SIMRUNS \t\t= "				<< SIMRUNS);
	print(__FUNCTION__, "NUMBER_UES \t\t= "				<< NUMBER_UES);
	print(__FUNCTION__, "NUMBER_BSS \t\t= "				<< NUMBER_BSS);
	print(__FUNCTION__, "NUMBER_UAVS \t\t= "			<< NUMBER_UAVS);
	print(__FUNCTION__, "TXPOWER_BSENB \t\t= "			<< TXPOWER_BSENB);
	print(__FUNCTION__, "TXPOWER_UAV \t\t= "			<< TXPOWER_UAV);
	print(__FUNCTION__, "TXPOWER_UE \t\t= "				<< TXPOWER_UE);
	print(__FUNCTION__, "MAX_LONGRANGE_REF \t= "		<< MAX_LONGRANGE_REF);
	print(__FUNCTION__, "MAX_SHORTRANGE_REF \t= "		<< MAX_SHORTRANGE_REF);
	print(__FUNCTION__, "INITIAL_ENERGY \t\t= "			<< INITIAL_ENERGY);
	print(__FUNCTION__, "INITIAL_BATT_VOLTAGE \t= "		<< INITIAL_BATT_VOLTAGE);
	print(__FUNCTION__, "RANDOMSEED \t\t= "				<< RANDOMSEED);
	print(__FUNCTION__, "ENABLE_UAVS \t\t= "			<< ENABLE_UAVS);
	print(__FUNCTION__, "AUTO_DEPLOYMENT \t= "			<< AUTO_DEPLOYMENT);


	ns3::RngSeedManager::SetSeed(RANDOMSEED);


	Ptr<ListPositionAllocator> UEPosition = CreateObject<ListPositionAllocator> ();
	UEPosition = generatePositionAllocator(5e2, NUMBER_UES, "random", 1.5);
	
	UE_NODECONTAINER.Create(NUMBER_UES);
	MobilityHelper mobilityUE;
	mobilityUE.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobilityUE.SetPositionAllocator(UEPosition);
	mobilityUE.Install(UE_NODECONTAINER);
	

	// Install Mobility Model ENB
	BS_NODECONTAINER.Create(NUMBER_BSS);
	// Ptr<ListPositionAllocator> BSPosition;
	Ptr<ListPositionAllocator> BSPosition = CreateObject<ListPositionAllocator> ();
	BSPosition->Add(Vector3D(5e3-1.1e3,5e3-1.1e3,30));
	// BSPosition->Add(Vector3D(5e3+1.1e3,5e3+1.1e3,30));

	MobilityHelper mobilityBS;
	mobilityBS.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobilityBS.SetPositionAllocator(BSPosition);
	mobilityBS.Install(BS_NODECONTAINER);



	Ptr<ListPositionAllocator> UAVPosition = CreateObject<ListPositionAllocator> ();
	UAVPosition = load_random_positions(UE_NODECONTAINER, DRONE_ALTITUDE, &NUMBER_UAVS);
	// Install Mobility Model UAV
	UAV_NODECONTAINER.Create(NUMBER_UAVS);
	MobilityHelper mobilityUAV;
	mobilityUAV.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
	mobilityUAV.SetPositionAllocator(UAVPosition);
	mobilityUAV.Install(UAV_NODECONTAINER);


	EPC_HELPER = CreateObject<PointToPointEpcHelper>();
	LTE_HELPER = CreateObject<LteHelper> ();
	LTE_HELPER->SetEpcHelper (EPC_HELPER);
	LTE_HELPER->SetSchedulerType ("ns3::RrFfMacScheduler");
	LTE_HELPER->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	LTE_HELPER->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(100));
	LTE_HELPER->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
	
	Ptr<Node> pgw = EPC_HELPER->GetPgwNode ();
	
	// Create a single RemoteHost
	NodeContainer remoteHostContainer;
	remoteHostContainer.Create (1);
	Ptr<Node> remoteHost = remoteHostContainer.Get (0);
	InternetStackHelper internet;
	internet.Install (remoteHostContainer);
	
	// Create the Internet
	PointToPointHelper p2ph;
	p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
	p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
	p2ph.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (10)));
	NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
	Ipv4AddressHelper ipv4h;
	ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
	Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
	Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1); // interface 0 is localhost, 1 is the p2p device

	Ptr<Ipv4StaticRouting> remoteHostStaticRouting = IPV4ROUTING_HELPER.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
	remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

	NodeContainer ueNodes = UE_NODECONTAINER;

	ENB_NODECONTAINER.Add(BS_NODECONTAINER);
	if (ENABLE_UAVS){
		ENB_NODECONTAINER.Add(UAV_NODECONTAINER);
	}

	// ==== ENERGY ====
	UavMobilityEnergyModelHelper energy_helper;
	energy_helper.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
	energy_helper.Set("AscendEnergyConversionFactor",DoubleValue(25.0));
	energy_helper.Set("DescendEnergyConversionFactor",DoubleValue(20.9));
	energy_helper.Set("MoveEnergyConversionFactor",DoubleValue(22.0));
	energy_helper.Set("HoverCurrent",DoubleValue(21.1));

	if (!ENABLE_UAVS){
		INITIAL_ENERGY = 0;
	}
	energy_helper.SetEnergySource("ns3::LiIonEnergySource",
		"LiIonEnergySourceInitialEnergyJ", DoubleValue (INITIAL_ENERGY),
		"InitialCellVoltage", DoubleValue(23),
		"NominalCellVoltage", DoubleValue(22.2),
		"ExpCellVoltage", DoubleValue(20.2),
		"ThresholdVoltage", DoubleValue(22.2)
		// "RatedCapacity", DoubleValue(5000), // 8
	);
	UAV_ENERGY_MODE = energy_helper.Install (UAV_NODECONTAINER);

	CONNECTIONS				= std::vector<std::vector<int>>(ENB_NODECONTAINER.GetN(),			std::vector<int> (ueNodes.GetN(), -1));

	std::stringstream prefix_stream;
	prefix_stream
		<< "[" << SIMTIME << " s]"
		<< "[" << NUMBER_UES << " UEs]"
		<< "[" << NUMBER_BSS << " BSs]"
		<< "[" << NUMBER_UAVS << " UAVs]"
		<< "[" << INITIAL_ENERGY << " J]"
		<< "";
	PARAMS_PREFIX = prefix_stream.str().c_str();
	print(__FUNCTION__, PARAMS_PREFIX);






	// CONFIGURING LTE AND INTERNET STACK
	Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));
	Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue(false));
	Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue(false));
	
	Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (true));
	Config::SetDefault ("ns3::LteHelper::UseCa", BooleanValue (true));
	Config::SetDefault ("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue (2));
	Config::SetDefault ("ns3::LteHelper::EnbComponentCarrierManager", StringValue ("ns3::RrComponentCarrierManager"));


	// Install LTE Devices in eNB and UEs
	Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (TXPOWER_BSENB));
	LTE_HELPER->SetEnbDeviceAttribute("DlBandwidth", UintegerValue (BANDWIDTH_BS_ENB_DL)); //Set Download BandWidth
	LTE_HELPER->SetEnbDeviceAttribute("UlBandwidth", UintegerValue (BANDWIDTH_BS_ENB_UP)); //Set Upload Bandwidth
	BS_LTEDEVS = LTE_HELPER->InstallEnbDevice (BS_NODECONTAINER);

	Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (TXPOWER_UAV));
	LTE_HELPER->SetEnbDeviceAttribute("DlBandwidth", UintegerValue (BANDWIDTH_UAV_ENB_DL)); //Set Download BandWidth
	LTE_HELPER->SetEnbDeviceAttribute("UlBandwidth", UintegerValue (BANDWIDTH_UAV_ENB_UP)); //Set Upload Bandwidth
	UAV_LTEDEVS = LTE_HELPER->InstallEnbDevice (UAV_NODECONTAINER);

	if (ENABLE_UAVS)
		ENB_LTEDEVS = NetDeviceContainer(UAV_LTEDEVS, BS_LTEDEVS);
	else
		ENB_LTEDEVS = NetDeviceContainer(BS_LTEDEVS);
	
	double noiseFigure = 9.0; // noise figure in dB
	Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue(TXPOWER_UE) );         // Transmission power in dBm
	Config::SetDefault ("ns3::LteUePhy::NoiseFigure", DoubleValue(noiseFigure) );     // Default 5
	UE_LTEDEVS = LTE_HELPER->InstallUeDevice (ueNodes);

	LTE_HELPER->SetAttribute("PathlossModel", StringValue("ns3::HybridBuildingsPropagationLossModel"));

	if (LOADB_ENABLED){
		LTE_HELPER->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm"); // disable automatic handover
	}else{
		LTE_HELPER->SetHandoverAlgorithmType ("ns3::A2A4RsrqHandoverAlgorithm"); // Handover by Reference Signal Reference Power (RSRP)
		LTE_HELPER->SetHandoverAlgorithmAttribute ("ServingCellThreshold", UintegerValue(30)); //default: 30
		LTE_HELPER->SetHandoverAlgorithmAttribute ("NeighbourCellOffset", UintegerValue(1)); //default: 1
	}







	// Install the IP stack on the UEs
	internet.Install (ueNodes);
	Ipv4InterfaceContainer ueIpIfaces = EPC_HELPER->AssignUeIpv4Address (UE_LTEDEVS);

	for (unsigned int i = 0; i < ueNodes.GetN(); ++i){
		Ptr<Node> ue = ueNodes.Get (i); 
		Ptr<NetDevice> ueLteDevice = UE_LTEDEVS.Get (i);

		// set the default gateway for the UE
		Ptr<Ipv4StaticRouting> ueStaticRouting = IPV4ROUTING_HELPER.GetStaticRouting (ue->GetObject<Ipv4> ());          
		ueStaticRouting->SetDefaultRoute (EPC_HELPER->GetUeDefaultGatewayAddress(), 1);

		IPV4ADDRESS_NODEID[ueIpIfaces.GetAddress (i)] = ue->GetId();
		// print(__FUNCTION__, "UE ["<< ue->GetId() <<"] got ipv4 address: " << ueIpIfaces.GetAddress (i));
	}

	LTE_HELPER->Attach(UE_LTEDEVS);
	LTE_HELPER->AddX2Interface(ENB_NODECONTAINER);










	/* SETTING THE UE APPLICATION DATARATES */
	double app_datarate = 1e6; // bps
	for (unsigned i = 0; i < UE_NODECONTAINER.GetN(); ++i){
		UE_DATARATE[UE_NODECONTAINER.Get (i)->GetId()] = app_datarate;
	}

	/* Creating the CBR application */
	uint16_t port = 9090;
	Packet::EnablePrinting ();
	for (unsigned i = 0; i < ueNodes.GetN(); ++i){
		Ptr<Node> ue = ueNodes.Get (i);
		port++;

		Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
		startTimeSeconds->SetAttribute ("Min", DoubleValue (1.01));
		startTimeSeconds->SetAttribute ("Max", DoubleValue (2.99));

		OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(remoteHostAddr, port));
		onoff.SetAttribute("OnTime",				StringValue("ns3::ConstantRandomVariable[Constant=1]"));
		onoff.SetAttribute("OffTime",				StringValue("ns3::ConstantRandomVariable[Constant=0]"));
		onoff.SetAttribute("EnableSeqTsSizeHeader",	BooleanValue(true));

		onoff.SetConstantRate (DataRate (to_string(UE_DATARATE[ueNodes.Get (i)->GetId()])+"bps"));

		Time startTime = Seconds (startTimeSeconds->GetValue ());
		ApplicationContainer apps = onoff.Install(ue);
		
		apps.Start(startTime);
		apps.Stop(Seconds(SIMTIME));

		PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(remoteHostAddr, port));
		sink.Install(remoteHostContainer);
	}




	// intall flow monitor and get stats
	Ptr<FlowMonitor> monitor = FLOWMON_HELPER.InstallAll();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(FLOWMON_HELPER.GetClassifier());

	monitor->SetAttribute ("StartTime",					TimeValue (Seconds (0)));
	Simulator::Schedule(Seconds(1), &QoSMonitor, monitor, 1);
	Simulator::Schedule(Seconds(0), &SaveNodesPosition, Seconds(1));
	Simulator::Schedule(Seconds(0), &ShowStatus, 1);



	// FILLING NODE MAP
	for(unsigned i = 0; i < BS_NODECONTAINER.GetN(); i++){
		NODESMAP[BS_NODECONTAINER.Get(i)->GetId()] = "BS";
	}
	for(unsigned i = 0; i < UE_NODECONTAINER.GetN(); i++){
		NODESMAP[UE_NODECONTAINER.Get(i)->GetId()] = "UE";
		IS_POORLY_ASSISTED[UE_NODECONTAINER.Get(i)->GetId()] = false;
		UE_ENB_CONNECTION[UE_NODECONTAINER.Get(i)->GetId()] = -1;
	}
	for(unsigned i = 0; i < UAV_NODECONTAINER.GetN(); i++){
		NODESMAP[UAV_NODECONTAINER.Get(i)->GetId()] = "UAV";
		UAV_NUMBERCONNECTIONS[UAV_NODECONTAINER.Get(i)->GetId()] = 0;
	}
	NODESMAP[remoteHost->GetId()] = "RM";
	NODESMAP[pgw->GetId()] = "PGW";

	Ipv4InterfaceContainer allIpInterfaces;
	allIpInterfaces.Add(internetIpIfaces);
	allIpInterfaces.Add(ueIpIfaces);
	for(unsigned i = 0; i < allIpInterfaces.GetN(); i++){
		int nodeid = allIpInterfaces.Get(i).first->GetNetDevice(allIpInterfaces.Get(i).second)->GetNode()->GetId();
		IPV4ADDRESS_NODEID[allIpInterfaces.GetAddress (i)] = nodeid;
	}







	for(unsigned i = 0; i < BS_NODECONTAINER.GetN(); i++){
		for(unsigned j = 0; j < UE_NODECONTAINER.GetN(); j++){
			IS_CONNECTION[UE_NODECONTAINER.Get(j)->GetId()][BS_NODECONTAINER.Get(i)->GetId()] = false;
			NEIGHBORS_MAP[BS_NODECONTAINER.Get(i)->GetId()][UE_NODECONTAINER.Get(j)->GetId()] = -DBL_MAX;
		}
	}
	for(unsigned i = 0; i < UAV_NODECONTAINER.GetN(); i++){
		for(unsigned j = 0; j < UE_NODECONTAINER.GetN(); j++){
			IS_CONNECTION[UE_NODECONTAINER.Get(j)->GetId()][UAV_NODECONTAINER.Get(i)->GetId()] = false;
			NEIGHBORS_MAP[UAV_NODECONTAINER.Get(i)->GetId()][UE_NODECONTAINER.Get(j)->GetId()] = -DBL_MAX;
		}
	}
	for(unsigned i = 0; i < UE_NODECONTAINER.GetN(); i++){
		PairdictRsrp newPair;
		newPair.enb_nodeid = -1;
		newPair.rsrp = -DBL_MAX;
		BEST_UE_RSRP[UE_NODECONTAINER.Get(i)->GetId()] = newPair;
	}


	Config::Connect("/NodeList/*/$ns3::psc::UavMobilityEnergyModel/EnergyDepleted", MakeCallback(&EnergyDepleted));
	Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback(&CourseChange));

	Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Drop", MakeCallback(&Drop));

	/* SIGNAL REPORTING CALLBACKS */
	// Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",	MakeCallback(&NotifyConnectionEstablishedEnb));
	// Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport",	MakeCallback(&RecvMeasurementReportCallback));


	/* CALLBACKS (eNBs)*/
	// Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RrcTimeout", MakeCallback(&RrcTimeoutEnb));

	/* CALLBACKS (UEs)*/
	Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/ReportUeMeasurements",	MakeCallback(&ReportUeMeasurements));
	// Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr", MakeCallback(&ReportCurrentCellRsrpSinr));

	/* LTE-RRC */
	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished", MakeCallback(&NotifyConnectionEstablishedUe));
	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/InitialCellSelectionEndError", MakeCallback(&InitialCellSelectionEndError));
	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessError", MakeCallback(&RandomAccessError));
	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionTimeout", MakeCallback(&ConnectionTimeout));
	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError", MakeCallback(&HandoverEndError));
	// Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionReconfiguration", MakeCallback(&UEConnectionReconfiguration));

	/* LTE-MAC */
	// Config::Connect("/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/ComponentCarrierMapUe/*/LteUeMac/RaResponseTimeout", MakeCallback (&NotifyRaResponseTimeoutUe));

	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk", MakeCallback(&NotifyHandoverEndOkUe));
	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart", MakeCallback(&NotifyHandoverStartUe));

	LTE_HELPER->EnableTraces ();
	Ptr<RadioBearerStatsCalculator> rlcStats = LTE_HELPER->GetRlcStats ();
	rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));



	// // ==== Radio Environment Map Helper ====
	// print(__FUNCTION__, "\u001b[31m rem.out ENABLED!!!\u001b[0m");
	// Ptr<RadioEnvironmentMapHelper> remHelper;
	// remHelper = CreateObject<RadioEnvironmentMapHelper> ();
	// // remHelper->SetAttribute ("ChannelPath", StringValue ("/ChannelList/3/$ns3::SpectrumChannel"));
	// remHelper->SetAttribute ("ChannelPath", StringValue ("/ChannelList/3"));
	// remHelper->SetAttribute ("OutputFile", StringValue ("rem.out"));
	// remHelper->SetAttribute ("XMin", DoubleValue (3.5e3));
	// remHelper->SetAttribute ("XMax", DoubleValue (6.5e3));
	// remHelper->SetAttribute ("XRes", UintegerValue (1.3e3));
	// remHelper->SetAttribute ("YMin", DoubleValue (3.5e3));
	// remHelper->SetAttribute ("YMax", DoubleValue (6.5e3));
	// remHelper->SetAttribute ("YRes", UintegerValue (1.3e3));
	// remHelper->SetAttribute ("Z", DoubleValue (1.5));
	// // remHelper->SetAttribute ("StopWhenDone", BooleanValue (false));
	// remHelper->Install ();
	// // ======================================






	// AnimationInterface animator("lte_animation.xml");

	// std::string nodeId_str = "";
	// animator.SetMobilityPollInterval(Seconds(10));

	// for (uint32_t i = 0; i < BS_NODECONTAINER.GetN(); ++i){
	// 	nodeId_str = std::to_string(BS_NODECONTAINER.Get(i)->GetId());
	// 	animator.UpdateNodeDescription( BS_NODECONTAINER.Get(i), "BS["+nodeId_str+"](" + std::to_string(i) + ")");
	// 	animator.UpdateNodeColor(BS_NODECONTAINER.Get(i), 0, 128, 255);
	// 	animator.UpdateNodeSize(BS_NODECONTAINER.Get(i)->GetId(), 150, 150);
	// }
	// for (uint32_t i = 0; i < ueNodes.GetN(); ++i){
	// 	nodeId_str = std::to_string(ueNodes.Get(i)->GetId());
	// 	animator.UpdateNodeDescription(ueNodes.Get(i), "UE["+nodeId_str+"](" + std::to_string(i) + ")");
	// 	animator.UpdateNodeColor(ueNodes.Get(i), 204, 204, 0);
	// 	animator.UpdateNodeSize(ueNodes.Get(i)->GetId(), 150, 150);
	// }
	// for (uint32_t i = 0; i < UAV_NODECONTAINER.GetN(); ++i){
	// 	nodeId_str = std::to_string(UAV_NODECONTAINER.Get(i)->GetId());
	// 	animator.UpdateNodeDescription(UAV_NODECONTAINER.Get(i), "UAV["+nodeId_str+"](" + std::to_string(i) + ")");
	// 	animator.UpdateNodeColor(UAV_NODECONTAINER.Get(i), 204, 0, 204);
	// 	animator.UpdateNodeSize(UAV_NODECONTAINER.Get(i)->GetId(), 150, 150);
	// }

	// nodeId_str = std::to_string(pgw->GetId());
	// animator.UpdateNodeDescription(pgw, "PGW["+nodeId_str+"](" + std::to_string(0) + ")");
	// animator.UpdateNodeColor(pgw, 100, 10, 0);
	// animator.UpdateNodeSize(pgw->GetId(), 100, 100);

	// nodeId_str = std::to_string(remoteHost->GetId());
	// animator.UpdateNodeDescription(remoteHost, "remoteHost["+nodeId_str+"](" + std::to_string(0) + ")");
	// animator.UpdateNodeColor(remoteHost, 200, 10, 0);
	// animator.UpdateNodeSize(remoteHost->GetId(), 100, 100);



	std::string simulation_finished_filename = "simulation_finished.txt";
	std::ofstream simulation_finished_stream;

	Simulator::Stop(Seconds (SIMTIME+1));
	Simulator::Run();
	Simulator::Destroy();

	simulation_finished_stream.open(simulation_finished_filename, std::ios::app);
	simulation_finished_stream << "SIMULATION FINISHED SUCCESSFULY";
	
	print(__FUNCTION__, "\u001b[32m" << "SIMULATION FINISHED SUCCESSFULY\u001b[0m");
	std::stringstream exec_cmd;

	exec_cmd << "mkdir uav_lte_results"
		<< " 2>/dev/null ";
	exec(exec_cmd.str().c_str());
	exec_cmd.str(std::string());
	
	exec_cmd << "mv *.txt uav_lte_results/"
		<< " 2>/dev/null ";
	exec(exec_cmd.str().c_str());
	exec_cmd.str(std::string());

	exec_cmd << "mv *.csv uav_lte_results/"
		<< " 2>/dev/null ";
	exec(exec_cmd.str().c_str());
	exec_cmd.str(std::string());

	print(__FUNCTION__, "\u001b[32m" << "Files saved at \u001b[33m uav_lte_results\u001b[0m");
	
	return 0;
}

