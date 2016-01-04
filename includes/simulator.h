#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "dllmacro.h"
#include "mapobject.h"
#include "agentbuilder.h"
#include "planmanager.h"
#include "RVO2/RVO.h"

namespace POICS{
	class POICS_API Simulator{
	public:
		static double AGENT_RADIUS;
		static double AGENT_GOAL_SQUARE;
		static double AGENT_MAXSPEED;
		static double AGENT_TIMEHORIZON;
		static double AGENT_TIMEHORIZONOBS;
		static double AGENT_NEIGHBORDIST;
		
		Simulator(){}
		virtual ~Simulator(){}

		virtual double getTimestep() const = 0;
		virtual const AgentList& getActiveAgents() const = 0;

		virtual void initialize(double deltaTimestep) = 0;
		virtual void update() = 0;
		virtual bool finished() = 0;

		static Simulator* create (MapArea& _maparea, AgentBuilder& _agentbuilder, PlanManager& _planner);
	};
}

#endif