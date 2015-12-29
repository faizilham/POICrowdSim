#ifndef MAPCOMPILER_H
#define MAPCOMPILER_H

#include "shapes.h"
#include "mapobject.h"
#include "graph.h"

namespace POICS {

	class PathFinder {
	private:
		std::vector<Polygon>* corridors;
	public:
		PathFinder(){}
		~PathFinder(){}

		void setCorridors(std::vector<Polygon>& _corridors){ corridors = &_corridors;}

		void getPath(const Point& start, const Point& end, std::vector<Point>& result_path){
			// TODO pathfinding algo and move to other file
		}
	};

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

		void getPath(const Point& start, const Point& end, std::vector<Point>& result_path){
			pathfinder.getPath(start, end, result_path);
		}

		double getLength(const Point& start, const Point& end);
	};

	class AStarAbstractGraph {
	public:
		NodeSet nodes; EdgeSet edges;
		int spawnNodeIdStart, exitNodeIdStart, poiNodeIdStart;

		AStarAbstractGraph(MapArea& _maparea, HMNavMesh& hmnav);
		~AStarAbstractGraph();
	};	
}

#endif