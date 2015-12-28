#ifndef MAPCOMPILER_H
#define MAPCOMPILER_H

#include "shapes.h"
#include "mapobject.h"

namespace POICS {

	class HMNavMeshGenerator {
	private:
		MapArea& maparea;
	public:
		HMNavMeshGenerator(MapArea& _maparea): maparea(_maparea){}
		~HMNavMeshGenerator(){}
		void buildNavMesh(std::vector<Polygon>& result_navmesh);
	};

	/*class PathFinder {
	public:
		std::vector<Polygon>& corridors;
		PathFinder(std::vector<Polygon>& _corridors): corridors(_corridors){};
		virtual ~PathFinder(){}

		virtual void getPath(const Point& start, const Point& end, std::vector<Point>& result_path) = 0;
		virtual double getLength(const Point& start, const Point& end) = 0;
	};*/

	class AStarAbstractGraph {
	public:
		AStarAbstractGraph(int num_poi, int num_spawn, int num_exit);
		~AStarAbstractGraph();
	};	
}

#endif