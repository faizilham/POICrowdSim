#ifndef COMPILEDMAP_H
#define COMPILEDMAP_H

#include "dllmacro.h"
#include "shapes.h"
#include "mapobject.h"
#include "graph.h"
#include "pathfinder.h"
#include <list>

namespace POICS {
	/* Hertel-Mehlhorn convex polygon partition based navigation mesh */
	class POICS_API HMNavMesh {
	private:
		std::vector<Polygon> corridors;
		PathFinder& pathfinder;
	public:
		HMNavMesh(PathFinder& _pathfinder): pathfinder(_pathfinder){
			pathfinder.setCorridors(corridors);
		}

		~HMNavMesh(){}
		void build(MapArea& maparea);

		std::vector<Polygon>& getCorridors(){ return corridors;}

		void getPath(const Point& start, int startCorridor, const Point& end, int endCorridor, double agentWidth, std::vector<Point>& result_path) const;

		double getDistance(const Point& start, int startCorridor, const Point& end, int endCorridor, double agentWidth) const;

		int findCorridor(const Point& p) const;
	};

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

		void buildPlan(int distance_budget, std::vector<double>& topic_interest, std::list<int>& result_plan) const;

		// build route from Point a to b, without a in the resulting list
		void buildNextRoute(Point& from, int nodeTo, std::list<Point>& result_path) const;
		void buildNextRoute(Point& from, Point& to, std::list<Point>& result_path) const;
		Point getRandomPoint(int node) const;
	};	
}

#endif