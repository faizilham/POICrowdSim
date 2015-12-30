#ifndef COMPILEDMAP_H
#define COMPILEDMAP_H

#include "shapes.h"
#include "mapobject.h"
#include "graph.h"
#include "pathfinder.h"

namespace POICS {
	/* Hertel-Mehlhorn convex polygon partition based navigation mesh */
	class HMNavMesh {
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

	class AStarAbstractGraph {
	public:
		NodeSet nodes; EdgeSet edges;
		int spawnNodeIdStart, exitNodeIdStart, poiNodeIdStart;

		std::vector<Point> nodePosition;
		std::vector<int> nodeCorridorId;

		AStarAbstractGraph(MapArea& _maparea, HMNavMesh& hmnav);
		~AStarAbstractGraph(){}
	};	
}

#endif