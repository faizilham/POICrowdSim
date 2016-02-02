#ifndef NAVMESH_H
#define NAVMESH_H

#include "dllmacro.h"
#include "shapes.h"
#include "mapobject.h"
#include "pathfinder.h"
#include "agentbuilder.h"

namespace POICS{

	enum class CornerSmoothing {NONE, PORTAL, POLYOFFSET};

	/* Hertel-Mehlhorn convex polygon partition based navigation mesh */
	class POICS_API HMNavMesh {
	private:
		std::vector<Polygon> corridors;
		PathFinder& pathfinder;

		bool trianglenavmesh, makelane; CornerSmoothing smoothing;
	public:
		HMNavMesh(PathFinder& _pathfinder, bool _triangle, bool _makelane, CornerSmoothing _smoothing)
		: pathfinder(_pathfinder), trianglenavmesh(_triangle), makelane(_makelane), smoothing(_smoothing){
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