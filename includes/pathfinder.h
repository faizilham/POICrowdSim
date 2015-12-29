#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "shapes.h"
#include <vector>

namespace POICS {

	class PathFinder {
	private:
		std::vector<Polygon>* corridors;
	public:
		PathFinder(){}
		~PathFinder(){}

		void setCorridors(std::vector<Polygon>& _corridors){ corridors = &_corridors;}
		void getPath(const Point& start, int startCorridor, const Point& end, int endCorridor, std::vector<Point>& result_path);
	};
}

#endif