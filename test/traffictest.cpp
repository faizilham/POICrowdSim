#include "navmesh.h"
#include "planmanager.h"
#include "simulator.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <exception>
#include <cstdlib>
#include <cstring>
#include <string>
#include <memory>
#include <cmath>
#include "rng.h"

using namespace POICS;
using namespace std;

double scorefunc(const NodeSet& nodes, std::vector<double>& topic_param, const std::vector<int>& path){
	double sum  = 0.0;

	int n = path.size();
	for (int i = 0; i < n; ++i){
		int node = path[i];

		for (int j = 0; j < nodes.num_score_elmts; ++j){
			sum += nodes.getScoreElement(node,j) * topic_param[j];
		}
	}

	return sum;
}

double spfunc(const NodeSet& nodes, std::vector<double>& topic_param, const std::vector<int>& path, int newNode){
	double sum  = 0.0;

	int n = path.size();
	for (int i = 0; i < n; ++i){
		int node = path[i];

		for (int j = 0; j < nodes.num_score_elmts; ++j){
			sum += nodes.getScoreElement(node,j) * topic_param[j];
		}
	}

	for (int j = 0; j < nodes.num_score_elmts; ++j){
		sum += nodes.getScoreElement(newNode,j) * topic_param[j];
	}

	return sum;
}

int main(int argc, char** argv){
	try{

		if (argc < 2) {
			exit(1);
		}

		/**** Parse Settings ****/
		cout << setprecision(5);

		bool trianglenavmesh = false;
		bool makelane = true;
		CornerSmoothing smoothing = CornerSmoothing::POLYOFFSET;

		if (argc > 2){
			for (int i = 2; i < argc; ++i){
				string arg = argv[i];

				if (arg == "--seed") {
					if (i == argc - 1) exit(1);
					string arg2 = argv[i+1];

					unsigned int seed = (unsigned int) stoi(arg2);
					RNG::setRandomSeed(seed);
					
					i = i + 1;
				} else if (arg == "--triangle") {
					trianglenavmesh = true;
				} else if (arg == "--nolane") {
					makelane = false;
				} else if (arg == "--portal") {
					smoothing = CornerSmoothing::PORTAL;
				} else if (arg == "--nosmooth") {
					smoothing = CornerSmoothing::NONE;
				} else {
					cout<<"Unknown option: "<<arg<<"\n";
				}
			}
		}

		/**** Load File and Build Components ****/
		string mapfile = string(argv[1]) + string(".xarea");
		string agentfile = string(argv[1]) + string(".xprof");

		MapArea m;
		m.loadFromXML(mapfile.c_str());
		
		AgentBuilder as(m.getTopicIds());
		as.loadFromXML(agentfile.c_str());

		PathFinder pf;
		HMNavMesh hm(pf, trianglenavmesh, makelane, smoothing);
		hm.build(m);

		PlanManager pm(m, hm);

		/**** Initialize Simulator ****/
		std::unique_ptr<Simulator> sim(Simulator::create(m, as, pm));
		cerr<<"Calculating Plan...\n";
		sim->initialize(1);
		

		/**** Do Simulation ****/
		cerr<<"\nSimulating...\n";
		while (!sim->finished()){
			sim->update();
			cerr << "\rTimesteps: " << (int) sim->getTimestep() << flush;
		}

		cerr << "\n";

		/**** Post-Simulation ****/
		for (Agent* agent : sim->getFinishedAgents()){
			double RMS = sqrt(agent->totalVelocity / agent->walkingTimesteps);
			double totalTime = agent->endTime - agent->startTime;

			cout	<< agent->id << " "
					<< agent->profile_name << " "
					<< agent->startTime << " "
					<< totalTime << " "
					<< agent->walkingTimesteps << " "
					<< agent->poiTimesteps << " "
					<< RMS << " "
					<< agent->duration << " "
					<< agent->totalpoi << " "
					<< agent->metasolution.first << " "
					<< agent->metasolution.second << " "
					<< makelane
					<< "\n";
		}

	} catch (const exception& e) {
		cerr<<e.what();
	}

	return 0;
}