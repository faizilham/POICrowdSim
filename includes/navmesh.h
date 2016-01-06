#ifndef NAVMESH_H
#define NAVMESH_H

#include "dllmacro.h"
#include "shapes.h"
#include "mapobject.h"
#include "pathfinder.h"
#include "agentbuilder.h"

namespace POICS{
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

		int findCorridor(const Point& p);

		void calculateDensity(AgentList& agents, double radius);
	};
}


#endif