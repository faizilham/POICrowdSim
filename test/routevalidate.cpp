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

void brutePlanRecursive (NodeSet& nodes, EdgeSet& edges, std::vector<double>& topicInterest, int endNode, double budget, std::vector<int>& currentPath, double length, std::vector<int>& bestPath, double& bestScore, double& bestLen, std::vector<int>& candidates, std::vector<bool>& visit){
	int i = 0; bool added = false;

	for (int node : candidates){
		if (!visit[i]){
			
			double newlen = edges.getLength(currentPath.back(), node);
			double endlend = edges.getLength(node, endNode);

			if (length + newlen + endlend < budget){
				added = true;
				visit[i] = true;
				currentPath.push_back(node);

				brutePlanRecursive(nodes, edges, topicInterest, endNode, budget,  currentPath, length + newlen, bestPath, bestScore, bestLen, candidates, visit);
				
				currentPath.pop_back();
				visit[i] = false;
			} else {
				double s = scorefunc(nodes, topicInterest, currentPath);

				if (s > bestScore){
					bestScore = s; bestPath = currentPath; 
					bestLen = length + edges.getLength(currentPath.back(), endNode);
					bestPath.push_back(endNode); 
				}
			}
		}

		++i;
	}

	double sc = scorefunc(nodes, topicInterest, currentPath);

	if (!added && sc > bestScore){
		bestScore = sc; bestPath = currentPath;
		bestLen = length + edges.getLength(currentPath.back(), endNode);
		bestPath.push_back(endNode);
	}
}

void brutePlan(NodeSet& nodes, EdgeSet& edges,  Agent* agent, int poiStartIndex){
	std::vector<int> path, currentPath, candidates; double score = 0, length = 0; std::vector<bool> visit;

	std::vector<double>& topicInterest = agent->topicInterest;
	int start = agent->plan.front(), end = agent->plan.back();
	double budget = agent->duration;

	for (int i = poiStartIndex; i < nodes.num_nodes; ++i){
		if (abs(spfunc(nodes, topicInterest, path, i)) > 1e-6){ // if not zero interest
			candidates.push_back(i);
			visit.push_back(false);
		}
	}

	currentPath.push_back(start);
	brutePlanRecursive(nodes, edges, topicInterest, end, budget, currentPath, 0, path, score, length, candidates, visit);

	cout << agent->profile_name <<" "<<budget<<" ";

	cout<<score<<" "<<length<<" "<<path.size();

	cout<<" "<<agent->metasolution.first<<" "<<agent->metasolution.second<<" "<<agent->plan.size();
}

void greedPlan (NodeSet& nodes, EdgeSet& edges,  Agent* agent, int poiStartIndex){
	std::vector<int> path, candidates; double score = 0, length = 0; std::vector<bool> visit;

	std::vector<double>& topicInterest = agent->topicInterest;
	int start = agent->plan.front(), end = agent->plan.back();
	double budget = agent->duration;

	for (int i = poiStartIndex; i < nodes.num_nodes; ++i){
		if (abs(spfunc(nodes, topicInterest, path, i)) > 1e-6){ // if not zero interest
			candidates.push_back(i);
			visit.push_back(false);
		}
	}

	path.push_back(start); int next, node; double curS;
	int num_nodes = candidates.size();

	do {
		next = -1;
		curS = 0;
		for (int i = 0; i < num_nodes; ++i){
			if (visit[i]) continue;

			node = candidates[i];

			double newlen = edges.getLength(path.back(), node);
			double endlend = edges.getLength(node, end);

			double sc = spfunc(nodes, topicInterest, path, node);

			if ((length + newlen + endlend < budget) && (curS < sc)){
				next = i;
				curS = sc;
			}
		}

		if (next > -1){
			visit[next] = true;
			node = candidates[next];
			length += edges.getLength(path.back(), node);
			path.push_back(node);
		}

	} while (next > -1);

	length += edges.getLength(path.back(), end);
	path.push_back(end);

	score = scorefunc(nodes, topicInterest, path);

	cout<<" "<<score<<" "<<length<<" "<<path.size()<< "\n";
}

void calculateGoldPlans (PlanManager& pm, std::unique_ptr<Simulator>& sim) {

	NodeSet& nodes = pm.getNodes(); EdgeSet& edges = pm.getEdges();
	int poiStartIndex = pm.poiNodeIdStart;

	/*for (int i = 0; i < nodes.num_nodes; ++i){
		for (int j = 0; j < nodes.num_nodes; ++j){
			cout << edges.getLength(i,j) << " ";
		}		
		cout << endl;
	}*/

	for (Agent* agent : sim->getInitialAgents()){
		brutePlan(nodes, edges, agent, poiStartIndex);
		greedPlan(nodes, edges, agent, poiStartIndex);
	}
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

		// rate plan
		cerr<<"Rate Plan...\n";
		calculateGoldPlans (pm, sim);
		

		/**** Do Simulation ****/
		/*cerr<<"\nSimulating...\n";
		while (!sim->finished()){
			sim->update();
			cerr << "\rTimesteps: " << (int) sim->getTimestep() << flush;
		}

		cerr << "\n";*/

		/**** Post-Simulation ****/
		/*for (Agent* agent : sim->getFinishedAgents()){
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
					<< agent->metasolution.second
					<< "\n";
		}*/

	} catch (const exception& e) {
		cerr<<e.what();
	}

	return 0;
}