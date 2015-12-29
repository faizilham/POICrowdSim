#include "pathfinder.h"

#include <cstdlib>
#include <cmath>
#include "priority_queue.h"

namespace POICS{

	AStarNode::AStarNode(Polygon* _poly)
	: closed(false), opened(false), from(NULL), gvalue(INFINITY), hvalue(INFINITY), polygon(_poly) {
		id = polygon->id;
	}

	double heuristic(Polygon* current, Polygon* target){
		return current->center().squareDistanceTo(target->center());
	}

	void buildPortal(AStarNode* last, std::vector<AStarNode>& anodes, std::vector<Portal*>& result_portal){
		std::vector<Polygon*> reverse_route;
	
		while(last != NULL){
			reverse_route.push_back(last->polygon);
			last = last->from;
		}

		int n = reverse_route.size();

		for (int i = n - 1; i > 0; --i){
			Polygon *from = reverse_route[i], *to = reverse_route[i-1];

			result_portal.push_back(from->getNeighbor(to->id));
		}
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
				buildPortal(current, anodes, result_portal);
				return true;
			}

			openset.pop();
			current->closed = true;
			closedset.push_back(current);
			for (const Portal& portal : current->polygon->getNeighbors()){
				AStarNode* neighbor = &(anodes[portal.to_id]);

				if (neighbor->closed) continue;

				double curr_gvalue = current->gvalue + portal.roughDistance;

				if (!neighbor->opened){
					openset.push(neighbor->hvalue, neighbor);
					neighbor->opened = true;
				} else if (curr_gvalue >= neighbor->gvalue){
					continue;
				}

				neighbor->from = current;
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