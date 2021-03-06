#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "dllmacro.h"
#include "shapes.h"
#include <vector>

namespace POICS {

	class AStarNode {
	public:
		int id;
		bool closed, opened;
		AStarNode* from;
		double gvalue;
		double hvalue;
		Polygon* polygon;


		AStarNode(Polygon* _poly);

		bool operator==(const AStarNode& rhs){
			return id == rhs.id;
		}

	};

	class POICS_API PathFinder {
	private:
		std::vector<Polygon>* corridors;

		bool corridorAStar(const Point& start, int startCorridor, const Point& end, int endCorridor, double agentWidth, std::vector<Portal>& result_portal) const;

	public:
		PathFinder(){}
		~PathFinder(){}

		void setCorridors(std::vector<Polygon>& _corridors){ corridors = &_corridors;}
		void getPath(const Point& start, int startCorridor, const Point& end, int endCorridor, double agentWidth, std::vector<Point>& result_path) const;
	};
}

#endif