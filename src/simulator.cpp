#include "simulator.h"
#include <random>
#include <cmath>

#include <iostream>

namespace POICS{
	static std::random_device rd;
	static std::mt19937 sim_rng(rd());
	static const double PI = 3.14159265358979323846;
	static const double AGENT_RADIUS = 2.0;
	static const double AGENT_GOAL_SQUARE = 2.5 * 2.5;
	static const double AGENT_MAXSPEED = 2.0;
	static const RVO::Vector2 IDENTITY(0.0, 0.0);

	void agentCleaner(AgentList& agents){
		for (auto itr = agents.begin(); itr != agents.end(); ++itr){
			Agent *agent = *itr;
			delete agent;
		}
		agents.clear();
	}

	Simulator::~Simulator(){
		agentCleaner(initialAgents);
		agentCleaner(activeAgents);
		agentCleaner(exitAgents);
	}

	void Simulator::buildObstacles(){
		

		for (Polygon& obstacle : maparea->getObstacles()){
			std::vector<RVO::Vector2> rvoobstacle;
			for (Point& point : obstacle.getPoints()){
				rvoobstacle.emplace_back(point.x, point.y);
			}

			rvo.addObstacle(rvoobstacle);
		}

		rvo.processObstacles();
	}

	RVO::Vector2 toRVOVector(Point& p){
		return RVO::Vector2(p.x, p.y);
	}

	Point toPoint(const RVO::Vector2& vec){
		return Point(vec.x(), vec.y());
	}

	RVO::Vector2 initPos(int id){
		return RVO::Vector2(-100.0 - (id * 10.0), -100);
	}

	void Simulator::initialize(double _deltaTimestep){
		deltaTimestep = _deltaTimestep;
		maxTimestep = maparea->timesteps;
		
		currentTimestep = 0.0;
		rvo.setTimeStep(deltaTimestep);

		// build obstacles
		buildObstacles();

		// build agents
		agentbuilder->generateAgents(maxTimestep, initialAgents);
		num_agents = agentbuilder->getNumAgents();

		// neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed
		// from ExampleRoadmap: 15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f
		rvo.setAgentDefaults(20.0f, 10, 5.0f, 2.0f, AGENT_RADIUS, AGENT_MAXSPEED);

		for (Agent* agent : initialAgents){
			rvo.addAgent(initPos(agent->id));
		}
	}

	// normalized velocity with perturbation based on current and next
	RVO::Vector2 prefVelocity(RVO::Vector2 curr, RVO::Vector2 next){
		std::uniform_real_distribution<double> rnd_rad(0.0, PI);
		std::uniform_real_distribution<double> rnd_len(0.0, 0.0002f);
		float angle = rnd_rad(sim_rng);
		float dist = rnd_len(sim_rng);

		return RVO::normalize(next - curr) + dist * RVO::Vector2(std::cos(angle), std::sin(angle));
	}

	void Simulator::update(){
		currentTimestep += deltaTimestep;

		rvo.doStep();

		for (auto itr = activeAgents.begin(); itr != activeAgents.end(); ){
			auto oldItr = itr++;
		 	Agent* agent = *oldItr;

		 	switch(agent->state){
		 		case AgentState::INIT: agent->nextState(); break;
		 		case AgentState::EXITING:{
		 			// TODO edit this erase part when parallelize
		 			rvo.setAgentPosition(agent->id, initPos(agent->id));
		 			exitAgents.push_back(agent);
		 			activeAgents.erase(oldItr);
		 		} break;
		 		case AgentState::TO_POI: {
		 			agent->position = toPoint(rvo.getAgentPosition(agent->id));

		 			if (agent->position.squareDistanceTo(agent->route.front()) < AGENT_GOAL_SQUARE){
		 				agent->route.pop_front();
		 				if (!agent->route.empty()){

		 					// set velocity to the next point in route
		 					RVO::Vector2 currPos = toRVOVector(agent->position);
							RVO::Vector2 nextPos = toRVOVector(agent->route.front());

			 				rvo.setAgentPrefVelocity (agent->id, prefVelocity(currPos, nextPos));	
		 				} else {
		 					agent->currentNode = agent->plan.front(); agent->plan.pop_front();
		 					if (agent->nextState() == AgentState::IN_POI){
		 						// TODO set nextUpdate based on place duration & activity type
		 						agent->nextUpdate = currentTimestep + 10.0;
		 					}

		 					rvo.setAgentPrefVelocity (agent->id, IDENTITY);
		 				}
		 			} else {
		 				RVO::Vector2 currPos = toRVOVector(agent->position);
						RVO::Vector2 nextPos = toRVOVector(agent->route.front());
						RVO::Vector2 prefV = prefVelocity(currPos, nextPos);

		 				rvo.setAgentPrefVelocity (agent->id, prefV);
		 			}
		 		} break;
		 		case AgentState::IN_POI: {
		 			if (currentTimestep > agent->nextUpdate){
		 				// TODO activity type
		 				agent->nextState();

		 				// build next route
		 				int next = agent->plan.front();
						planner->buildNextRoute(agent->position, next, agent->route);
		 				
		 				// set velocity to the next point in route
	 					RVO::Vector2 currPos = toRVOVector(agent->position);
						RVO::Vector2 nextPos = toRVOVector(agent->route.front());

		 				rvo.setAgentPrefVelocity (agent->id, prefVelocity(currPos, nextPos));
		 			}
		 		}
		 		default: break;
		 	}


		}

		// agents that aren't yet enter simulation
		for (auto itr = initialAgents.begin(); itr != initialAgents.end(); ){
			auto oldItr = itr++;
		 	Agent* agent = *oldItr;

			if (currentTimestep < agent->nextUpdate) continue; // not yet the spawn time, continue

			planner->buildPlan(agent->duration * 60 * AGENT_MAXSPEED, agent->topicInterest, agent->plan);
			int start = agent->plan.front(); agent->plan.pop_front(); 
			int next = agent->plan.front();
			
			agent->position = planner->getRandomPoint(start);
			planner->buildNextRoute(agent->position, next, agent->route);

			RVO::Vector2 currPos = toRVOVector(agent->position);
			RVO::Vector2 nextPos = toRVOVector(agent->route.front());

			rvo.setAgentPosition(agent->id, currPos);
			rvo.setAgentPrefVelocity (agent->id, prefVelocity(currPos, nextPos));

			/* TODO edit this part (erase and move) when parallelize */
			activeAgents.push_back(agent);
			initialAgents.erase(oldItr);
		}
	}

	bool Simulator::finished(){
		return (currentTimestep >= maxTimestep); // && (activeAgents.empty());
	}
}