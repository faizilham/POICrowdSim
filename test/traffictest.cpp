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
#include "clipper/clipper.h"

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

bool compAgentId(Agent* a, Agent* b) { return a->id < b->id; }

static double CLIPPER_SCALE = 1000.0;

void toClipperPath(Polygon& poly, ClipperLib::Path& path){
	for (Point& point : poly.getPoints()){
		path << ClipperLib::IntPoint((int) (point.x * CLIPPER_SCALE), (int) (point.y * CLIPPER_SCALE));
	}
}

void fromClipperPath(ClipperLib::Path& path, Polygon& poly){
	poly.reset();
	for (ClipperLib::IntPoint p : path){
		poly.addPoint(p.X / CLIPPER_SCALE, p.Y / CLIPPER_SCALE);
	}

	poly.calcCentroid(); //poly.setOrientation()
}

void merge(std::vector<Polygon>& obstacles, std::vector<Polygon>& newObstacles){
	//, Polygon& areaResult, std::vector<Polygon>& obstacleResult
	ClipperLib::ClipperOffset co; ClipperLib::Paths offsets;
	// offset
	
	for (Polygon& obstacle : obstacles){
		ClipperLib::Path path;
		toClipperPath(obstacle, path);
		offsets.push_back(path);
	}
	ReversePaths(offsets);

	// merge
	ClipperLib::Clipper clipper; ClipperLib::Paths solution;

	clipper.AddPaths(offsets, ClipperLib::ptSubject, true);
	clipper.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

	for (ClipperLib::Path& p : solution){
		Polygon newObs; fromClipperPath(p, newObs);
		newObstacles.push_back(newObs);
	}
}

void CountObs(MapArea& m){
	std::vector<Polygon> newObstacles;

	merge(m.getObstacles(), newObstacles);

	double total = 0;

	for (Polygon& p : newObstacles){
		total += p.getArea();
	}

	cout << newObstacles.size() << " " << total << endl;
}

int main(int argc, char** argv){
	try{

		if (argc < 2) {
			exit(1);
		}

		/**** Parse Settings ****/
		cout << setprecision(5);

		bool trianglenavmesh = false;
		bool rmsonly = false;
		bool makelane = true;
		int numagentparam = 0;
		int timelimit = 9999999;
		bool countobs = false;
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
				} else if (arg == "--rmsonly") {
					rmsonly = true;
				} else if (arg == "--numagent") {
					if (i == argc - 1) exit(1);
					string arg2 = argv[i+1];

					numagentparam = stoi(arg2);
					
					i = i + 1;
				} else if (arg == "--limit") {
					if (i == argc - 1) exit(1);
					string arg2 = argv[i+1];

					timelimit = stoi(arg2);
					
					i = i + 1;
				} else if (arg == "--countobs") {
					countobs = true;
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

		if (countobs){
			CountObs(m);
			return 0;
		}
		
		AgentBuilder as(m.getTopicIds());
		as.loadFromXML(agentfile.c_str());

		if (numagentparam > 0) as.setNumAgents(numagentparam);

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
		while (!sim->finished() && (sim->getTimestep() < timelimit)){
			sim->update();
			cerr << "\rTimesteps: " << (int) sim->getTimestep() << flush;
		}

		cerr << "\n";

		/**** Post-Simulation ****/

		sim->getFinishedAgents().sort(compAgentId);

		for (Agent* agent : sim->getFinishedAgents()){
			double RMS = sqrt(agent->totalVelocity / agent->walkingTimesteps);
			double totalTime = agent->endTime - agent->startTime;

			if (rmsonly){
				cout << RMS << "\n";
			} else {
				cout << agent->id << " "
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
			
		}

	} catch (const exception& e) {
		cerr<<e.what();
	}

	return 0;
}