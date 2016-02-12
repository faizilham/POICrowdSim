#include "simulator.h"
#include <random>
#include "rng.h"
#include <cmath>
#include <iostream>
#include "omp.h"

namespace POICS{
	static const double PI = 3.14159265358979323846;
	static const RVO::Vector2 IDENTITY(0.0, 0.0);

	double Simulator::AGENT_RADIUS = 1.0;
	double Simulator::AGENT_GOAL_SQUARE = 2;
	double Simulator::AGENT_MAXSPEED = 1.0;
	double Simulator::AGENT_TIMEHORIZON = 2.0;
	double Simulator::AGENT_TIMEHORIZONOBS = 1.0;
	double Simulator::AGENT_NEIGHBORDIST = 15.0;


	class POICS_API SimulatorImpl : public Simulator{
	private:
		RVO::RVOSimulator rvo;
		MapArea* maparea;
		AgentBuilder* agentbuilder;
		PlanManager* planner;
		double currentTimestep, maxTimestep, deltaTimestep, recalcTimestep, maxRecalcTimestep;
		int num_agents;

		
		void buildObstacles();
	public:
		AgentList initialAgents, activeAgents, exitAgents;
		
		SimulatorImpl(MapArea* _maparea, AgentBuilder* _agentbuilder, PlanManager* _planner)
		: maparea(_maparea), agentbuilder(_agentbuilder), planner(_planner){}

		virtual ~SimulatorImpl();

		virtual double getTimestep() const { return currentTimestep;}
		virtual const AgentList& getActiveAgents() const {return activeAgents;}
		virtual const AgentList& getFinishedAgents() const {return exitAgents;}

		virtual void initialize(double deltaTimestep);
		virtual void update();
		virtual bool finished();
	};

	Simulator* Simulator::create (MapArea& _maparea, AgentBuilder& _agentbuilder, PlanManager& _planner){
		return new SimulatorImpl(&_maparea, &_agentbuilder, &_planner);
	}

	void agentCleaner(AgentList& agents){
		for (auto itr = agents.begin(); itr != agents.end(); ++itr){
			Agent *agent = *itr;
			delete agent;
		}
		agents.clear();
	}

	SimulatorImpl::~SimulatorImpl(){
		agentCleaner(initialAgents);
		agentCleaner(activeAgents);
		agentCleaner(exitAgents);
	}

	void rectToRVOObst (Rect& r, std::vector<RVO::Vector2>& rvoobstacle){
		Polygon obstacle; rvoobstacle.clear();

		r.copyToPolygonCCW(obstacle);

		for (Point& point : obstacle.getPoints()){
			rvoobstacle.emplace_back(point.x, point.y);
		}
	}

	void SimulatorImpl::buildObstacles(){
		std::vector<RVO::Vector2> rvoobstacle;
		double d = 0.5; double w, h;
		w = maparea->width; h = maparea->height;

		for (Polygon& obstacle : maparea->getObstacles()){
			rvoobstacle.clear();
			for (Point& point : obstacle.getPoints()){
				rvoobstacle.emplace_back(point.x, point.y);
			}

			rvo.addObstacle(rvoobstacle);
		}

		
		Rect rtop(0, h+d, w, d), rbottom(0, 0, w, d), rleft(-d, h, d, h), rright(w+d, h, d, h);

		rectToRVOObst(rtop, rvoobstacle); rvo.addObstacle(rvoobstacle);
		rectToRVOObst(rbottom, rvoobstacle); rvo.addObstacle(rvoobstacle);
		rectToRVOObst(rleft, rvoobstacle); rvo.addObstacle(rvoobstacle);
		rectToRVOObst(rright, rvoobstacle); rvo.addObstacle(rvoobstacle);


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

	void SimulatorImpl::initialize(double _deltaTimestep){
		deltaTimestep = _deltaTimestep;
		maxTimestep = agentbuilder->entryTime;
		maxRecalcTimestep = 10.0;

		recalcTimestep = 0.0;
		currentTimestep = 0.0;
		rvo.setTimeStep(deltaTimestep);

		// build obstacles
		buildObstacles();

		// build agents
		agentbuilder->generateAgents(initialAgents);
		num_agents = agentbuilder->getNumAgents();

		// neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed
		// from ExampleRoadmap: 15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f
		rvo.setAgentDefaults(AGENT_NEIGHBORDIST, 10, AGENT_TIMEHORIZON, AGENT_TIMEHORIZONOBS, AGENT_RADIUS, AGENT_MAXSPEED);

		for (Agent* agent : initialAgents){
			rvo.addAgent(initPos(agent->id));

			agent->metasolution = planner->buildPlan(agent->duration * AGENT_MAXSPEED, agent->topicInterest, agent->plan);
			agent->totalpoi = agent->plan.size() - 2;
		}

		/*
		#pragma omp parallel
		#pragma omp single
		{
			for(auto it = initialAgents.begin(); it != initialAgents.end(); ++it){
				#pragma omp task firstprivate(it)
				{
					Agent* agent = *it;
					planner->buildPlan(agent->duration * AGENT_MAXSPEED, agent->topicInterest, agent->plan);
				}
			}
			#pragma omp taskwait
		}
		*/
	}

	// normalized velocity with perturbation based on current and next
	RVO::Vector2 prefVelocity(RVO::Vector2 curr, RVO::Vector2 next){
		std::uniform_real_distribution<double> rnd_rad(0.0, PI);
		std::uniform_real_distribution<double> rnd_len(0.0, 0.0001f);
		float angle = rnd_rad(sim_rng);
		float dist = rnd_len(sim_rng);

		return RVO::normalize(next - curr) + dist * RVO::Vector2(std::cos(angle), std::sin(angle));
	}

	void SimulatorImpl::update(){
		currentTimestep += deltaTimestep;
		recalcTimestep += deltaTimestep;

		rvo.doStep();

		for (auto itr = activeAgents.begin(); itr != activeAgents.end(); ){
			auto oldItr = itr++;
		 	Agent* agent = *oldItr;

		 	agent->position = toPoint(rvo.getAgentPosition(agent->id));

		 	switch(agent->state){
		 		case AgentState::INIT: agent->nextState(); break;
		 		case AgentState::EXITING:{
		 			// TODO edit this erase part when parallelize
		 			rvo.setAgentPosition(agent->id, initPos(agent->id));
		 			agent->endTime = currentTimestep;

		 			exitAgents.push_back(agent);
		 			activeAgents.erase(oldItr);
		 		} break;
		 		case AgentState::TO_POI: {
		 			RVO::Vector2 currVel = rvo.getAgentVelocity(agent->id);
		 			double currSquareSpeed = (currVel.x() * currVel.x()) + (currVel.y() * currVel.y());

		 			agent->totalVelocity += currSquareSpeed;
		 			agent->walkingTimesteps += deltaTimestep;

		 			if (agent->position.squareDistanceTo(agent->route.front()) < AGENT_GOAL_SQUARE){
		 				agent->nextUpdate = -1;
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
		 						double poiduration = maparea->getPois()[agent->currentNode - planner->poiNodeIdStart].activityTime;

		 						agent->nextUpdate = currentTimestep + poiduration;
		 						agent->poiTimesteps += poiduration;
		 					}

		 					//agent->identityPosition = agent->position;

		 					rvo.setAgentPrefVelocity (agent->id, IDENTITY);
		 				}
		 			} else {

		 				Point cp = agent->position;
		 				Point np = agent->route.front();

		 				RVO::Vector2 currPos = toRVOVector(cp);
						RVO::Vector2 nextPos = toRVOVector(np);

						auto second = ++(agent->route.begin());

		 				if (!rvo.queryVisibility(currPos, nextPos, AGENT_RADIUS)){

		 					if (agent->nextUpdate < 0) agent->nextUpdate = currentTimestep + 5.0;
		 					else if (agent->nextUpdate < currentTimestep) {

		 						double len = cp.distanceTo(np); Point unit;
								unit.x = (cp.x - np.x) / len;
								unit.y = (cp.y - np.y) / len;

								cp.x += AGENT_RADIUS * unit.x;
								cp.y += AGENT_RADIUS * unit.y;

			 					agent->route.clear();
			 					planner->buildNextRoute(cp, agent->plan.front(), agent->route);
			 					nextPos = toRVOVector(agent->route.front());
			 					agent->nextUpdate = -1;
			 				}
		 				} else if (second != agent->route.end()){
		 					RVO::Vector2 secondPos = toRVOVector(*second);
		 					double currNextDist = cp.squareDistanceTo(np);
		 					double nextSecDist = np.squareDistanceTo(*second);
		 					double currSecDist = cp.squareDistanceTo(*second);

		 					if ((currSecDist <= currNextDist + nextSecDist) && rvo.queryVisibility(currPos, secondPos, AGENT_RADIUS)){
		 						agent->route.pop_front(); nextPos = secondPos;
		 					}
		 				}

						RVO::Vector2 prefV = prefVelocity(currPos, nextPos);

		 				rvo.setAgentPrefVelocity (agent->id, prefV);
		 			}
		 		} break;
		 		case AgentState::IN_POI: {
		 			if (currentTimestep >= agent->nextUpdate){
		 				// TODO activity type
		 				agent->nextState();

		 				// build next route
		 				int next = agent->plan.front();
						planner->buildNextRoute(agent->position, next, agent->route);
		 				
		 				// set velocity to the next point in route
	 					RVO::Vector2 currPos = toRVOVector(agent->position);
						RVO::Vector2 nextPos = toRVOVector(agent->route.front());

		 				rvo.setAgentPrefVelocity (agent->id, prefVelocity(currPos, nextPos));
		 				agent->nextUpdate = -1;
		 			} /*else {
		 				RVO::Vector2 currPos = toRVOVector(agent->position);
						RVO::Vector2 nextPos = toRVOVector(agent->identityPosition);

		 				RVO::Vector2 prefV = prefVelocity(currPos, nextPos);

		 				rvo.setAgentPrefVelocity (agent->id, prefV);
		 			}*/
		 		}
		 		default: break;
		 	}

		 	if (recalcTimestep > maxRecalcTimestep){
		 		recalcTimestep = 0;
		 		//planner->getNavMesh()->calculateDensity(activeAgents, Simulator::AGENT_RADIUS);
		 	}
		}

		// agents that aren't yet enter simulation
		for (auto itr = initialAgents.begin(); itr != initialAgents.end(); ){
			auto oldItr = itr++;
		 	Agent* agent = *oldItr;

			if (currentTimestep < agent->nextUpdate) continue; // not yet the spawn time, continue

			int start = agent->plan.front(); agent->plan.pop_front(); 
			int next = agent->plan.front();
			
			agent->position = planner->getRandomPoint(start);
			planner->buildNextRoute(agent->position, next, agent->route);

			RVO::Vector2 currPos = toRVOVector(agent->position);
			RVO::Vector2 nextPos = toRVOVector(agent->route.front());

			rvo.setAgentPosition(agent->id, currPos);
			rvo.setAgentPrefVelocity (agent->id, prefVelocity(currPos, nextPos));

			agent->currentNode = start;
			agent->startTime = currentTimestep;

			/* TODO edit this part (erase and move) when parallelize */
			activeAgents.push_back(agent);
			initialAgents.erase(oldItr);
		}
	}

	bool SimulatorImpl::finished(){
		return (activeAgents.empty() && initialAgents.empty()); // || (currentTimestep >= maxTimestep) 
	}
}