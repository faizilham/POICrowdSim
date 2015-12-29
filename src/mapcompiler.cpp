#include "mapcompiler.h"
#include "polypartition/polypartition.h"
#include "helper.h"
#include <list>
#include <cstdio>
#include <iostream>

using std::list;

namespace POICS {
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

	void createTPPList(MapArea& maparea, std::list<TPPLPoly>& tpls){
		TPPLPoly tpl; tpls.clear();

		/** add map area polygon **/
		Rect area(0, 0, maparea.width, maparea.height);
		Polygon parea; area.copyToPolygonCCW(parea);

		toTPPLPoly(parea, false, tpl);
		tpls.push_back(tpl);

		/** add obstacles **/
		for (Polygon& poly : maparea.getObstacles()){
			toTPPLPoly(poly, true, tpl);
			tpls.push_back(tpl);
		}
	}

	void HMNavMesh::build(MapArea& maparea){

		TPPLPartition pp;
		std::list<TPPLPoly> input, output;

		/* convert to TPPLPoly */
		createTPPList(maparea, input);

		/* HM Partition */
		if (!pp.ConvexPartition_HM(&input, &output)){
			except("Invalid input polygon when building corridors");
		}

		/* convert back to POICS::Polygon */
		corridors.clear();
		int i = 0; Polygon poly;
		for (TPPLPoly& tpl : output){
			toPOICSPoly(tpl, poly);
			poly.id = i; corridors.push_back(poly); ++i;
		}

		/* setup neighbor list */
		Point p1, p2;
		int n = corridors.size();
		for (int i = 0; i < n - 1; ++i){
			for (int j = i + 1; j < n; ++j){
				Polygon& poly1 = corridors[i];
				Polygon& poly2 = corridors[j];

				if (poly1.testNeighborhood(poly2, p1, p2)){
					poly1.addNeighbor(poly2, p1, p2);
					poly2.addNeighbor(poly1, p1, p2);
				}
			}
		}

		/** test print **/
		std::cout<<n<<std::endl;
		for (Polygon& pl: corridors){
			std::cout<<pl.id<<":";
			for (Portal& portal : pl.getNeighbors()){
				std::cout<<portal.neighbor->id<<" ";
			}
			std::cout<<std::endl;
		}
	}

	double HMNavMesh::getLength(const Point& start, const Point& end){
		std::vector<Point> path;

		pathfinder.getPath(start, end, path);

		if (path.size() < 2) return 0;

		auto p1 = path.begin();
		double sum = 0;

		for (auto p2 = path.begin() + 1; p2 != path.end(); ++p2){
			sum += p1->distanceTo(*p2);
			p1 = p2;
		}

		return sum;
	}

	AStarAbstractGraph::AStarAbstractGraph(MapArea& maparea, HMNavMesh& hmnav){
		std::vector<POI>& pois = maparea.getPois();
		std::vector<SpawnPoint>& spawns = maparea.getSpawns();
		std::vector<ExitPoint>& exits = maparea.getExits();

		int num_topic = maparea.getTopics().size();

		/** build nodes **/
		int num_nodes = pois.size() + spawns.size() + exits.size();
		nodes.init(num_nodes, num_topic);
		edges.init(num_nodes);

		spawnNodeIdStart = 0;
		exitNodeIdStart = spawns.size();
		poiNodeIdStart = exitNodeIdStart + exits.size();

		std::vector<Point> nodePos;

	}
}
