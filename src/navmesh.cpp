#include "navmesh.h"
#include "polypartition/polypartition.h"
#include "clipper/clipper.h"
#include "helper.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace POICS{

	static double CLIPPER_SCALE = 1000.0;

	void toClipperPath(Polygon& poly, ClipperLib::Path& path){
		for (Point& point : poly.getPoints()){
			path << ClipperLib::IntPoint((int) (point.x * CLIPPER_SCALE), (int) (point.y * CLIPPER_SCALE));
		}
	}

	void fromClipperPath(ClipperLib::Path& path, Polygon& poly){
		poly.reset();
		for (ClipperLib::IntPoint p : path){
			poly.addPoint(p.X / CLIPPER_SCALE, p.Y / CLIPPER_SCALE);
		}

		poly.calcCentroid(); //poly.setOrientation()
	}

	void offsetAndMerge(Polygon& area, std::vector<Polygon>& obstacles, Polygon& newArea, std::vector<Polygon>& newObstacles){
		//, Polygon& areaResult, std::vector<Polygon>& obstacleResult
		ClipperLib::ClipperOffset co; ClipperLib::Paths offsets;
		// offset
		
		for (Polygon& obstacle : obstacles){
			ClipperLib::Path path; toClipperPath(obstacle, path); ClipperLib::Paths newPath;

			co.AddPath(path, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
			co.Execute(newPath, 1.5 * CLIPPER_SCALE);
			co.Clear();

			offsets.push_back(newPath.front());
		}
		ReversePaths(offsets);

		// merge
		ClipperLib::Path areaPath; toClipperPath(area, areaPath);
		ClipperLib::Clipper clipper; ClipperLib::PolyTree solution;

		clipper.AddPath(areaPath, ClipperLib::ptSubject, true);
		clipper.AddPaths(offsets, ClipperLib::ptClip, true);
		clipper.Execute(ClipperLib::ctDifference, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);


		ClipperLib::PolyNode* areaNode = solution.GetFirst();
		if (areaNode != NULL){
			fromClipperPath(areaNode->Contour, newArea);

			for (ClipperLib::PolyNode* holes : areaNode->Childs){
				Polygon newObs; fromClipperPath(holes->Contour, newObs);
				newObstacles.push_back(newObs);
			}
		}
	}

	void toTPPLPoly(Polygon& poly, bool hole, TPPLPoly& tpl){
		std::vector<Point>& points = poly.getPoints();
		tpl.Init(points.size());

		int i = 0;
		for (Point& p : points){
			tpl[i].x = p.x; tpl[i].y = p.y;
			++i;
		}

		if(hole) {
			tpl.SetHole(true);
			tpl.SetOrientation(TPPL_CW);
		} else {
			tpl.SetOrientation(TPPL_CCW);
		}
	}

	void toPOICSPoly(TPPLPoly& tpl, Polygon& poly){
		poly.reset();

		int numpoints = tpl.GetNumPoints();
		for (int i = 0; i < numpoints; ++i){
			poly.addPoint(tpl[i].x, tpl[i].y);
		}
	}


	void createTPPList(MapArea& maparea, std::list<TPPLPoly>& tpls, std::vector<Polygon>& specials){
		TPPLPoly tpl; tpls.clear();

		/** add map area polygon **/
		Rect area(0, 0, maparea.width, maparea.height);
		Polygon parea; area.copyToPolygonCCW(parea);


		Polygon newArea;
		std::vector<Polygon> newObs;


		offsetAndMerge(parea, maparea.getObstacles(), newArea, newObs);

		toTPPLPoly(newArea, false, tpl);
		tpls.push_back(tpl);

		/** add obstacles **/
		for (Polygon& poly : newObs){
			toTPPLPoly(poly, true, tpl);
			tpls.push_back(tpl);
		}

		/*for (SpawnPoint& special : maparea.getSpawns()){
			special.border.copyToPolygonCW(parea);
			toTPPLPoly(parea, true, tpl);
			tpls.push_back(tpl);
			specials.push_back(parea);
		}

		for (ExitPoint& special : maparea.getExits()){
			special.border.copyToPolygonCW(parea);
			toTPPLPoly(parea, true, tpl);
			tpls.push_back(tpl);
			specials.push_back(parea);
		}

		for (POI& special : maparea.getPois()){
			special.border.copyToPolygonCW(parea);
			toTPPLPoly(parea, true, tpl);
			tpls.push_back(tpl);
			specials.push_back(parea);
		}*/
	}

	void HMNavMesh::build(MapArea& maparea){

		TPPLPartition pp;
		std::list<TPPLPoly> input, output;
		std::vector<Polygon> specials;

		/* convert to TPPLPoly */
		createTPPList(maparea, input, specials);

		/* HM Partition */

		if (trianglenavmesh){
			if (!pp.Triangulate_EC(&input, &output)){
				except("Invalid input polygon when building corridors");
			}
		} else {
			if (!pp.ConvexPartition_HM(&input, &output)){
				except("Invalid input polygon when building corridors");
			}	
		}
		
		/* convert back to POICS::Polygon */
		// TPPL_CCW 1, TPPL_CW -1
		corridors.clear();
		int i = 0; Polygon poly;
		for (TPPLPoly& tpl : output){
			toPOICSPoly(tpl, poly);
			poly.calcCentroid();
			poly.id = i; corridors.push_back(poly); ++i;
		}

		for (Polygon& special : specials){
			special.calcCentroid();
			special.id = i; corridors.push_back(special); ++i;
		}

		/* setup neighbor list */
		Point p1, p2;
		int n = corridors.size();
		for (int i = 0; i < n - 1; ++i){
			for (int j = i + 1; j < n; ++j){
				Polygon& poly1 = corridors[i];
				Polygon& poly2 = corridors[j];

				// resulting polygon is CCW
				if (poly1.testNeighborhood(poly2, p1, p2, true)){
					double laneDiff = 2.5;

					poly1.addNeighbor(poly2, p1, p2); 
					
					// move right point a bit to left for making lane
					Portal& portal1 = poly1.getNeighbors().back();
					portal1.p2.x -= portal1.unit.x * laneDiff;
					portal1.p2.y -= portal1.unit.y * laneDiff;

					poly2.addNeighbor(poly1, p2, p1); // mirrored for neighbor

					// move right point a bit to left for making lane
					Portal& portal2 = poly2.getNeighbors().back();
					portal2.p2.x -= portal2.unit.x * laneDiff;
					portal2.p2.y -= portal2.unit.y * laneDiff;
				}
			}
		}
	}

	void HMNavMesh::getPath(const Point& start, int startCorridor, const Point& end, int endCorridor, double agentWidth, std::vector<Point>& result_path) const{
		pathfinder.getPath(start, startCorridor, end, endCorridor, agentWidth, result_path);
	}

	double calcDistance(std::vector<Point>& path){
		if (path.size() < 2) return 0.0;

		auto p1 = path.begin();
		double sum = 0.0;

		for (auto p2 = path.begin() + 1; p2 != path.end(); ++p2){
			sum += p1->distanceTo(*p2);
			p1 = p2;
		}

		return sum;
	}

	double HMNavMesh::getDistance(const Point& start, int startCorridor, const Point& end, int endCorridor, double agentWidth) const{
		std::vector<Point> path;

		pathfinder.getPath(start, startCorridor, end, endCorridor, agentWidth, path);

		return calcDistance(path);
	}


	int HMNavMesh::findCorridor(const Point& p) {
		int min_id = -1; double min_dist = INFINITY;
		for (Polygon& poly : corridors){
			if (poly.contains(p)){
				return poly.id;	
			} else {
				for (Point& p1 : poly.getPoints()){
					double dist = p.squareDistanceTo(p1);
					if (dist < min_dist){
						min_dist = dist;
						min_id = poly.id;
					}
				}
			}
		}

		return min_id;
	}

	void HMNavMesh::calculateDensity(AgentList& agents, double radius){
		int num_corridors = corridors.size();
		std::vector<int> num_agents(num_corridors);
		std::fill_n(num_agents.begin(), num_corridors, 0);

		for (Agent* agent : agents){
			if (agent->state != AgentState::TO_POI) continue;

			int c = findCorridor(agent->position);
			if (c == -1) continue;

			num_agents[c] += 1;
		}


		std::cout<<"dense ";
		for (int i = 0; i < num_corridors; ++i){
			corridors[i].calcDensityWeight(num_agents[i], radius);
			std::cout<<num_agents[i]<<":"<<corridors[i].getDensityWeight()<<" ";
		}
		std::cout<<std::endl;
	}

}