#include "planmanager.h"
#include <random>
#include <omp.h>
#include "gop.h"
#include "rng.h"
#include <iostream>

namespace POICS {
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

		std::vector<int> duration;

		startDistribution.reserve(spawns.size());
		nodePosition.reserve(num_nodes);
		nodeCorridorId.reserve(num_nodes);
		duration.reserve(num_nodes);

		for (SpawnPoint& spawn : spawns){
			Point center = spawn.border.center();
			nodePosition.push_back(center);
			startDistribution.push_back(spawn.dist);
			nodeCorridorId.push_back(hmnav->findCorridor(center));

			duration.push_back(0);
		}

		for (ExitPoint& ep : exits){
			Point center = ep.border.center();
			nodePosition.push_back(center);
			nodeCorridorId.push_back(hmnav->findCorridor(center));

			duration.push_back(0);
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

			duration.push_back(poi.activityTime);
		}

		#pragma omp parallel for firstprivate(num_nodes)
		for (int i = 0; i < num_nodes - 1; ++i){
			for (int j = i + 1; j < num_nodes; ++j){
				double distance = hmnav->getDistance(nodePosition[i], nodeCorridorId[i], nodePosition[j], nodeCorridorId[j], agentPathWidth);
				//edges.addEdgeSymmetric(i, j, distance);
				edges.addEdge(i, j, distance + duration[j]);
				edges.addEdge(j, i, distance + duration[i]);
			}
		}		
	}

	class WScoreFunction : public ScoreFunc{
	public:
		virtual double scorefunc(const NodeSet& nodes, std::vector<double>& topic_param, const std::vector<int>& path){
			double sum  = 0.0;
		
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
			double sum  = 0.0;

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

	SolutionMeta PlanManager::buildPlan(int distance_budget, std::vector<double>& topic_interest, std::list<int>& result_plan) const{
		int pi = 4, pt = 2000;

		// select start and end
		std::uniform_real_distribution<double> rnd(0.0, 1.0);

		int start = getRandomId(startDistribution, rnd(cm_rng));
		int end = (int) (rnd(cm_rng) * (poiNodeIdStart - exitNodeIdStart)) + exitNodeIdStart;

		WScoreFunction wsf;

		return two_param_iterative_gop(pi, pt, distance_budget, topic_interest, nodes, edges, poiNodeIdStart, start, end, wsf, result_plan);
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
