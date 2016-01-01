#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "mapobject.h"
#include "agentbuilder.h"
#include "compiledmap.h"
#include "RVO.h"

namespace POICS{

	class Simulator{
	private:
		RVO::RVOSimulator rvo;
		MapArea* maparea;
		AgentBuilder* agentbuilder;
		PlanManager* planner;
		double currentTimestep, maxTimestep, deltaTimestep;
		int num_agents;
		void buildObstacles();
	public:
		std::vector<AgentPtr> initialAgents;
		std::vector<AgentPtr> activeAgents;
		std::vector<AgentPtr> exitAgents;

		Simulator(MapArea& _maparea, AgentBuilder& _agentbuilder, PlanManager& _planner)
		: maparea(&_maparea), agentbuilder(&_agentbuilder), planner(&_planner){}

		~Simulator(){}

		void initialize(double deltaTimestep);
		void update();
		bool finished();
	};
}

#endif