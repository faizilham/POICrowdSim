#include "compiledmap.h"
#include "polypartition/polypartition.h"
#include "helper.h"
#include <list>
#include <cstdio>
#include <omp.h>
#include <iostream>

#include "imagehelper.h"

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

		// TPPL_CCW 1, TPPL_CW -1
		
		corridors.clear();
		int i = 0; Polygon poly;
		for (TPPLPoly& tpl : output){
			toPOICSPoly(tpl, poly);
			poly.calcCentroid();
			poly.id = i; corridors.push_back(poly); ++i;
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
					poly1.addNeighbor(poly2, p1, p2);
					poly2.addNeighbor(poly1, p2, p1); // mirrored for neighbor
				}
			}
		}

		/** test print **/
		/*std::cout<<n<<std::endl;
		for (Polygon& pl: corridors){

			std::cout<<pl.id<<":";
			for (Portal& portal : pl.getNeighbors()){
				std::cout<<portal.to_id<<" ";
			}
			std::cout<<std::endl<<pl<<" : "<<pl.center()<<std::endl;
		}*/
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


	int HMNavMesh::findCorridor(const Point& p) const{
		for (const Polygon& poly : corridors){
			if (poly.contains(p)) return poly.id;
		}

		return -1;
	}

	AStarAbstractGraph::AStarAbstractGraph(MapArea& maparea, HMNavMesh& hmnav){
		std::vector<POI>& pois = maparea.getPois();
		std::vector<SpawnPoint>& spawns = maparea.getSpawns();
		std::vector<ExitPoint>& exits = maparea.getExits();

		int num_topic = maparea.getTopics().size();

		// TODO set from agent manager object
		double agentPathWidth = 10.0; // actual agent width + some margin

		/** build nodes **/
		int num_nodes = pois.size() + spawns.size() + exits.size();
		nodes.init(num_nodes, num_topic);
		edges.init(num_nodes);

		spawnNodeIdStart = 0;
		exitNodeIdStart = spawns.size();
		poiNodeIdStart = exitNodeIdStart + exits.size();

		nodePosition.reserve(num_nodes);
		nodeCorridorId.reserve(num_nodes);

		for (SpawnPoint& spawn : spawns){
			Point center = spawn.border.center();
			nodePosition.push_back(center);
			nodeCorridorId.push_back(hmnav.findCorridor(center));
		}

		for (ExitPoint& ep : exits){
			Point center = ep.border.center();
			nodePosition.push_back(center);
			nodeCorridorId.push_back(hmnav.findCorridor(center));
		}

		int id = poiNodeIdStart;
		for (POI& poi : pois){
			Point center = poi.border.center();
			nodePosition.push_back(center);
			nodeCorridorId.push_back(hmnav.findCorridor(center));

			// set topic relevance
			for (int j = 0; j < num_topic; ++j){
				nodes.setScore(id, j, poi.topic_relevance[j]);
			}

			++id;
		}

		// print test
		/*std::cout<<"AStarAbstractGraph nodes"<<std::endl;
		for (int i = 0; i < num_nodes; ++i){
			std::cout<<nodePosition[i]<<" "<<nodeCorridorId[i]<<std::endl;

			for (int j = 0; j < num_topic; ++j){
				std::cout<<nodes.getScoreElement(i, j)<<" ";
			}

			std::cout<<std::endl;
		}*/

		/** calculate distances **/
		/*for (i = 0; i < num_nodes - 1; ++i){
			for (int j = i + 1; j < num_nodes; ++j){
				double distance = hmnav.getDistance(nodePosition[i], nodeCorridorId[i], nodePosition[j], nodeCorridorId[j]);
				edges.addEdgeSymmetric(i, j, distance);
			}
		}

		std::cout<<"AStarAbstractGraph edges"<<std::endl;
		for (int i = 0; i < num_nodes; ++i){
			for (int j = 0; j < num_nodes; ++j){
				std::cout<<edges.getLength(i, j)<<" ";
			}

			std::cout<<std::endl;
		}*/

		Painter painter(maparea.width, maparea.height, 0.5);

		// test draw
		painter.setColor(130, 130, 130);
		for (Polygon& pl : hmnav.getCorridors()) {
			painter.drawPoly(pl);
		}

		painter.setColor(255, 0, 0);
		for (SpawnPoint& spawn : spawns){
			painter.drawRect(spawn.border);
		}

		painter.setColor(0, 0, 255);
		for (ExitPoint& ep : exits){
			painter.drawRect(ep.border);
		}

		painter.setColor(0, 255, 0);
		for (POI& poi : pois){
			painter.drawRect(poi.border);
		}

		std::vector<std::vector<Point>> paths(num_nodes*num_nodes);

		/** calculate distances **/

		#pragma omp parallel for firstprivate(num_nodes)
		for (int i = 0; i < num_nodes - 1; ++i){
			for (int j = i + 1; j < num_nodes; ++j){
				std::vector<Point> path;
				hmnav.getPath(nodePosition[i], nodeCorridorId[i], nodePosition[j], nodeCorridorId[j], agentPathWidth, path);
				
				double distance = calcDistance(path);
				edges.addEdgeSymmetric(i, j, distance);

				paths.push_back(path);
			}
		}

		painter.setColor(255, 150, 0);
		for (std::vector<Point>& path : paths){
			int n = path.size();
			for (int i = 0; i < n-1; ++i){
				painter.drawLine(path[i], path[i+1]);
			}	
		}
		painter.save("tmp/test.bmp");
	}
}
