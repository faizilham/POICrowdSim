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
		return current->center().squareDistanceTo(target->center());// + target->getDensityWeight();
	}

	void buildPortal(AStarNode* last, std::vector<AStarNode>& anodes, double agentWidth, std::vector<Portal>& result_portal){
		std::vector<Polygon*> reverse_route;
	
		while(last != NULL){
			reverse_route.push_back(last->polygon);
			last = last->from;
		}

		int n = reverse_route.size();

		for (int i = n - 1; i > 0; --i){
			Polygon *from = reverse_route[i], *to = reverse_route[i-1];
			Portal portal = *from->getNeighbor(to->id);
			//Point& unit = portal.unit;

			//double halfMargin = (agentWidth + (portal.width / 2)) / 2;

			//std::uniform_real_distribution<double> randw(agentWidth, std::min(3*agentWidth, halfMargin));

			//double r1 = randw(pt_rng), r2 = randw(pt_rng);

			/*if (agentWidth > 0){
				double r1 = agentWidth / 2, r2 = agentWidth / 2;

				portal.p1.x += unit.x * r1;
				portal.p1.y += unit.y * r1;
				portal.p2.x -= unit.x * r2;
				portal.p2.y -= unit.y * r2;
			}*/

			result_portal.push_back(portal);
		}
	}

	bool PathFinder::corridorAStar(const Point& start, int startCorridor, const Point& end, int endCorridor, double agentWidth, std::vector<Portal>& result_portal) const{
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
				// build portal
				buildPortal(current, anodes, agentWidth, result_portal);
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
					/*if (portal.width < agentWidth){
						continue;
					}*/

					openset.push(neighbor->hvalue, neighbor);
					neighbor->opened = true;

					neighbor->from = current;
					neighbor->gvalue = curr_gvalue;
					neighbor->hvalue = curr_gvalue + heuristic(neighbor->polygon, target);
				} else if (curr_gvalue >= neighbor->gvalue){
					continue;
				} else {
					neighbor->from = current;
					neighbor->gvalue = curr_gvalue;
					double oldHVal = neighbor->hvalue;
					neighbor->hvalue = curr_gvalue + heuristic(neighbor->polygon, target);
					openset.update(oldHVal, neighbor->hvalue, neighbor);
				}

				
			}
		}

		return false;
	}

	void edgeMidPoint(const Point& start, const Point& end, std::vector<Portal>& portals, std::vector<Point>& path);
	void simpleFunnel(const Point& start, const Point& end, std::vector<Portal>& portals, std::vector<Point>& path);

	void PathFinder::getPath(const Point& start, int startCorridor, const Point& end, int endCorridor, double agentWidth, std::vector<Point>& result_path) const{
		// if same corridor, return
		if (startCorridor == endCorridor){
			result_path.push_back(start); result_path.push_back(end);
			return;
		}

		std::vector<Portal> result_portal;
		corridorAStar(start, startCorridor, end, endCorridor, agentWidth, result_portal);

		result_path.reserve(result_portal.size() + 2);
		simpleFunnel(start, end, result_portal, result_path);
	}

	double crossproduct(const Point& base, const Point& p1, const Point& p2) {
		double ax = p1.x - base.x;
		double ay = p1.y - base.y;
		double bx = p2.x - base.x;
		double by = p2.y - base.y;
		return bx*ay - ax*by;
	}

	void edgeMidPoint(const Point& start, const Point& end, std::vector<Portal>& portals, std::vector<Point>& path){
		path.push_back(start);

		for (Portal& portal : portals){
			path.push_back(portal.center);
		}

		path.push_back(end);
	}

	void simpleFunnel(const Point& start, const Point& end, std::vector<Portal>& portals, std::vector<Point>& path){
		/** ASSUMPTION: portal.p1 is right and p2 is left.
			It will be true as long as the portal constructed with points from testNeighborhood() **/
		Portal dummyEnd; dummyEnd.p1 = end; dummyEnd.p2 = end;
		portals.push_back(dummyEnd);

		int numportal = portals.size();
		Point funnelRight, funnelLeft, right, left, apex;
		int rightIdx = 0, leftIdx = 0, apexIdx = 0;

		apex = start;
		path.push_back(apex);

		funnelRight = portals[0].p2;
		funnelLeft = portals[0].p1;

		double product;

		for (int i = 1; i < numportal; ++i) {
			right = portals[i].p2;
			left = portals[i].p1;

			// update right

			product = crossproduct(apex, funnelRight, right);
			if (product < 0.0 || abs(product) < EPSILON) {
				if ((apex == funnelRight) || (crossproduct(apex, funnelLeft, right) > 0.0)) {
					// tighten funnel
					funnelRight = right;
					rightIdx = i;
				} else {
					// right cross left, so set left as new apex
					apex = funnelLeft;
					apexIdx = leftIdx;
					path.push_back(apex);

					// restart
					funnelLeft = apex;
					funnelRight = apex;
					leftIdx = apexIdx;
					rightIdx = apexIdx;
					i = apexIdx;

					continue;
				}
			}

			// update left
			product = crossproduct(apex, funnelLeft, left);
			if (product > 0.0 || abs(product) < EPSILON) {
				if ((apex == funnelLeft) || (crossproduct(apex, funnelRight, left) < 0.0)) {
					// tighten funnel
					funnelLeft = left;
					leftIdx = i;
				} else {
					// left cross right, so set right as new apex
					apex = funnelRight;
					apexIdx = rightIdx;
					path.push_back(apex);

					// restart
					funnelLeft = apex;
					funnelRight = apex;
					leftIdx = apexIdx;
					rightIdx = apexIdx;
					i = apexIdx;

					continue;
				}
			}
		}

		path.push_back(end);
	}

}