#include "compiledmap.h"
#include "polypartition/polypartition.h"
#include <random>
#include <algorithm>
#include <omp.h>
#include "helper.h"
#include "gop.h"

namespace POICS {
	static std::random_device rd;
	static std::mt19937 cm_rng(rd());

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

			// flip the saved obstacle points to CCW
			int n = tpl.GetNumPoints();
			for (i = 0; i < n; ++i){
				points[i].set(tpl[n - i - 1].x, tpl[n - i - 1].y);
			}
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

	PlanManager::PlanManager(MapArea& _maparea, HMNavMesh& _hmnav): maparea(&_maparea), hmnav(&_hmnav){
		std::vector<POI>& pois = maparea->getPois();
		std::vector<SpawnPoint>& spawns = maparea->getSpawns();
		std::vector<ExitPoint>& exits = maparea->getExits();

		int num_topic = maparea->getTopics().size();

		agentPathWidth = maparea->agentPathWidth; // actual agent width + some margin

		/** build nodes **/
		int num_nodes = pois.size() + spawns.size() + exits.size();
		nodes.init(num_nodes, num_topic);
		edges.init(num_nodes);

		spawnNodeIdStart = 0;
		exitNodeIdStart = spawns.size();
		poiNodeIdStart = exitNodeIdStart + exits.size();

		startDistribution.reserve(spawns.size());
		nodePosition.reserve(num_nodes);
		nodeCorridorId.reserve(num_nodes);

		for (SpawnPoint& spawn : spawns){
			Point center = spawn.border.center();
			nodePosition.push_back(center);
			startDistribution.push_back(spawn.dist);
			nodeCorridorId.push_back(hmnav->findCorridor(center));
		}

		for (ExitPoint& ep : exits){
			Point center = ep.border.center();
			nodePosition.push_back(center);
			nodeCorridorId.push_back(hmnav->findCorridor(center));
		}

		int id = poiNodeIdStart;
		for (POI& poi : pois){
			Point center = poi.border.center();
			nodePosition.push_back(center);
			nodeCorridorId.push_back(hmnav->findCorridor(center));

			// set topic relevance
			for (int j = 0; j < num_topic; ++j){
				nodes.setScore(id, j, poi.topic_relevance[j]);
			}

			++id;
		}

		#pragma omp parallel for firstprivate(num_nodes)
		for (int i = 0; i < num_nodes - 1; ++i){
			for (int j = i + 1; j < num_nodes; ++j){
				double distance = hmnav->getDistance(nodePosition[i], nodeCorridorId[i], nodePosition[j], nodeCorridorId[j], agentPathWidth);
				edges.addEdgeSymmetric(i, j, distance);
			}
		}		
	}

	class WScoreFunction : public ScoreFunc{
	public:
		virtual double scorefunc(const NodeSet& nodes, std::vector<double>& topic_param, const std::vector<int>& path){
			double sum  = 1.0;
		
			int n = path.size();
			for (int i = 0; i < n; ++i){
				int node = path[i];

				for (int j = 0; j < nodes.num_score_elmts; ++j){
					sum += nodes.getScoreElement(node,j) * topic_param[j];
				}
			}

			return sum;
		}

		virtual double spfunc(const NodeSet& nodes, std::vector<double>& topic_param, const std::vector<int>& path, int newNode){
			double sum  = 1.0;

			int n = path.size();
			for (int i = 0; i < n; ++i){
				int node = path[i];

				for (int j = 0; j < nodes.num_score_elmts; ++j){
					sum += nodes.getScoreElement(node,j) * topic_param[j];
				}
			}

			for (int j = 0; j < nodes.num_score_elmts; ++j){
				sum += nodes.getScoreElement(newNode,j) * topic_param[j];
			}

			return sum;
		}
	};


	int getRandomId(const std::vector<double>& distribution, double randomNum){
		double sum = 0; int i = 0;
		for (const double& d : distribution){
			sum += d;
			if (randomNum < sum) return i;
			++i;
		}

		return -1;
	}

	void PlanManager::buildPlan(int distance_budget, std::vector<double>& topic_interest, std::list<int>& result_plan) const{
		int pi = 3, pt = 2000;

		// select start and end
		std::uniform_real_distribution<double> rnd(0.0, 1.0);

		int start = getRandomId(startDistribution, rnd(cm_rng));
		int end = (int) (rnd(cm_rng) * (poiNodeIdStart - exitNodeIdStart)) + exitNodeIdStart;

		WScoreFunction wsf;

		two_param_iterative_gop(pi, pt, distance_budget, topic_interest, nodes, edges, poiNodeIdStart, start, end, wsf, result_plan);
	}

	void PlanManager::buildNextRoute(Point& from, int nodeTo, std::list<Point>& result_path) const{
		Point to = getRandomPoint(nodeTo);
		buildNextRoute(from, to, result_path);
	}

	void PlanManager::buildNextRoute(Point& from, Point& to, std::list<Point>& result_path) const{
		int corridorFrom = hmnav->findCorridor(from);
		int corridorTo = hmnav->findCorridor(to);

		std::vector<Point> vectorpath;

		hmnav->getPath(from, corridorFrom, to, corridorTo, agentPathWidth, vectorpath);

		result_path.clear();
		std::copy(vectorpath.begin(), vectorpath.end(), std::back_inserter(result_path));

		result_path.pop_front(); // remove start point from route
	}

	Point PlanManager::getRandomPoint(int node) const{
		if (node < exitNodeIdStart){ // spawn node
			SpawnPoint& sp = maparea->getSpawns()[node];
			return sp.border.getRandomPoint();
		} else if (node < poiNodeIdStart){
			ExitPoint& ep = maparea->getExits()[node - exitNodeIdStart];
			return ep.border.getRandomPoint();
		} else {
			POI& poi = maparea->getPois()[node - poiNodeIdStart];
			return poi.border.getRandomPoint();
		}
	}
}
