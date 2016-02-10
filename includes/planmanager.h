#ifndef COMPILEDMAP_H
#define COMPILEDMAP_H

#include "dllmacro.h"
#include "shapes.h"
#include "mapobject.h"
#include "graph.h"
#include "navmesh.h"
#include "helper.h"

#include <list>

namespace POICS {
	class POICS_API PlanManager {
	private:
		MapArea *maparea; HMNavMesh *hmnav;
		NodeSet nodes; EdgeSet edges;
		double agentPathWidth;
	public:
		int spawnNodeIdStart, exitNodeIdStart, poiNodeIdStart;

		std::vector<double> startDistribution;
		std::vector<Point> nodePosition;
		std::vector<int> nodeCorridorId;

		PlanManager(MapArea& _maparea, HMNavMesh& _hmnav);
		~PlanManager(){}

		SolutionMeta buildPlan(int distance_budget, std::vector<double>& topic_interest, std::list<int>& result_plan) const;

		// build route from Point a to b, without a in the resulting list
		void buildNextRoute(Point& from, int nodeTo, std::list<Point>& result_path) const;
		void buildNextRoute(Point& from, Point& to, std::list<Point>& result_path) const;
		Point getRandomPoint(int node) const;
		HMNavMesh* getNavMesh(){return hmnav;}
	};	
}

#endif