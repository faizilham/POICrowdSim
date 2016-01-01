#ifndef GOP_H
#define GOP_H

#include <vector>
#include <functional>
#include <list>
#include "graph.h"

/**
	Custom implementation of Two-Parameter Iterative Algorithm for Generalized Orienteering Problem (Silberholz & Golden, 2009)
**/

namespace POICS {
	typedef std::function<double(const NodeSet&, const std::vector<double>&, const std::vector<int>&)> scorefunc_t;
	typedef std::function<double(const NodeSet&, const std::vector<double>&, const std::vector<int>&, int)> spfunc_t;

	class ScoreFunc{
	public:
		virtual double scorefunc(const NodeSet& nodes, std::vector<double>& topic_param, const std::vector<int>& path) = 0;
		virtual double spfunc(const NodeSet& nodes, std::vector<double>& topic_param, const std::vector<int>& path, int newNode) = 0;
	};

	class Solution{
	public:
		double score, distance;
		int distance_budget;
		const NodeSet *nodes;
		const EdgeSet *edges;
		std::vector<double>* topic_param;
		std::vector<int> path;
		ScoreFunc* SF;

		Solution(int budget, std::vector<double>& _topic_param, const NodeSet* _nodes, const EdgeSet* _edges, ScoreFunc& _SF);
		~Solution();
		Solution(const Solution& sol);
		void operator= (const Solution& sol);
		void copy (const Solution& sol);
		void process_gop(int par_i, int par_t, int POIIdx, int start, int end);

	private:
		double countScore();
		double countSP(int newNode);

		double countDistance();

		void two_opt();
		void setNodeInMiddle(int pos, int node);
		int buildT(std::vector<int>& unused_nodes, bool* used);
		void buildUnused(std::vector<int>& unused_nodes, bool* used);
		void pathTightening(std::vector<int>& unused_nodes, bool* used);
	};

	void two_param_iterative_gop(int par_i, int par_t, int distance_budget, std::vector<double>& topic_param, const NodeSet& nodes, const EdgeSet& edges, int POIIdx, int start, int end, ScoreFunc& SF, std::list<int>& result);
}

#endif