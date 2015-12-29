#include "pathfinder.h"

#include <cmath>
#include "priority_queue.h"

namespace POICS{

	AStarNode::AStarNode(Polygon* _poly)
	: closed(false), opened(false), from(-1), gvalue(INFINITY), hvalue(INFINITY), polygon(_poly) {
		id = polygon->id;
	}

	double heuristic(Polygon* current, Polygon* target){
		return current->center().squareDistanceTo(target->center());
	}

	bool PathFinder::corridorAStar(const Point& start, int startCorridor, const Point& end, int endCorridor, std::vector<Portal*>& result_portal){
		std::vector<Polygon>& corridors = *(this->corridors);

		std::vector<AStarNode> anodes;
		std::vector<AStarNode*> closedset;
		PriorityQueue<double, AStarNode*> openset;

		Polygon* target = &(corridors[endCorridor]);

		for (Polygon& poly : corridors){
			anodes.emplace_back(&poly);	
		}

		AStarNode& startNode = anodes[startCorridor];
		startNode.gvalue = 0;
		startNode.hvalue = 0;

		openset.push(startNode.hvalue, &startNode);

		while (!openset.empty()){
			AStarNode* current = openset.front();
			
			if (current->id == endCorridor){
				// build path
				return true;
			}

			openset.pop();
			current->closed = true;
			closedset.push_back(current);
			for (const Portal& portal : current->polygon->getNeighbors()){
				AStarNode* neighbor = &(anodes[portal.to->id]);

				if (neighbor->closed) continue;

				double curr_gvalue = current->gvalue + portal.roughDistance;

				if (!neighbor->opened){
					openset.push(neighbor->hvalue, neighbor);
					neighbor->opened = true;
				} else if (curr_gvalue >= neighbor->gvalue){
					continue;
				}

				neighbor->from = current->id;
				neighbor->gvalue = curr_gvalue;
				neighbor->hvalue = curr_gvalue + heuristic(neighbor->polygon, target);
			}
		}

		return false;
	}

	void PathFinder::getPath(const Point& start, int startCorridor, const Point& end, int endCorridor, std::vector<Point>& result_path){
			// TODO pathfinding algo and move to other file
	}
}