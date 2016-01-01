#include "simulator.h"
#include <random>
#include <cmath>

namespace POICS{
	static std::random_device rd;
	static std::mt19937 sim_rng(rd());

	void Simulator::buildObstacles(){
		std::vector<RVO::Vector2> rvoobstacle; int i = 0;

		for (Polygon& obstacle : maparea->getObstacles()){
			rvoobstacle.clear();
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
		rvo.setAgentDefaults(37.5f, 10, 5.0f, 5.0f, 5.0f, 5.0f);

		double x = -100.0, y = -100.0, dx = -10.0;

		for (AgentPtr& agent : initialAgents){
			agent->pos.set(x, y); 
			rvo.addAgent(RVO::Vector2(x,y));
			x += dx;
		}
	}

	// normalized and perturbed velocity based on current and next
	RVO::Vector2 prefVelocity(RVO::Vector2 curr, RVO::Vector2 next){
		std::uniform_real_distribution<double> rnd_rad(0.0, M_PI);
		std::uniform_real_distribution<double> rnd_len(0.0, 0.0002f);
		float angle = rnd_rad(sim_rng);
		float dist = rnd_len(sim_rng);

		return RVO::normalize(next - curr) + dist * RVO::Vector2(std::cos(angle), std::sin(angle));
	}

	void Simulator::update(){
		int n;
		currentTimestep += deltaTimestep;

		for (auto itr = activeAgents.begin(); itr != activeAgents.end(); ++itr){
		 	AgentPtr& agent = *itr;

		 	// update RVO

		 	switch(agent->state){
		 		case AgentState::TO_POI: {

		 		}
		 		break;
		 		default: break;
		 	}

			// 	if (currentTimestep > agent->entryTimestep) continue;


		}

		// agents that aren't yet enter simulation

		n = initialAgents.size();
		for (int i = 0; i < n; ++i){
		 	AgentPtr& agent = initialAgents[i];

			if (currentTimestep < agent->nextUpdate) continue; // not yet the spawn time

			planner->buildPlan(agent->duration, agent->topic_interest, agent->plan);
			int start = agent->plan.front(); agent->plan.pop_front(); 
			int next = agent->plan.front();
			
			agent->position = planner->getRandomPoint(start);
			planner->buildNextRoute(agent->position, next, agent->route);

			RVO::Vector2 currPos = toRVOVector(agent->position);
			RVO::Vector2 nextPos = toRVOVector(agent->route.front());

			rvo.setAgentPosition(agent->id, currPos);
			rvo.setAgentPrefVelocity (agent->id, prefVelocity(currPos, nextPos))
		}
	}

	bool Simulator::finished(){
		return (currentTimestep >= maxTimestep) && (activeAgents.empty());
	}
}