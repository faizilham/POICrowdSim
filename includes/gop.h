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
	typedef std::function<int(const NodeSet&, const std::vector<double>&, const std::vector<int>&)> scorefunc_t;
	typedef std::function<int(const NodeSet&, const std::vector<double>&, const std::vector<int>&, int)> spfunc_t;

	class Solution{
	public:
		float score, distance;
		int distance_budget;
		const NodeSet *nodes;
		const EdgeSet *edges;
		std::vector<double>& topic_param;
		std::vector<int> path;
		scorefunc_t scorefunc;
		spfunc_t spfunc;

		Solution(int budget, std::vector<double>& _topic_param, const NodeSet* _nodes, const EdgeSet* _edges, scorefunc_t _scorefunc, spfunc_t _spfunc);
		~Solution();
		Solution(const Solution& sol);
		void operator= (const Solution& sol);
		void copy (const Solution& sol);
		void process_gop(int par_i, int par_t, int start, int end);

	private:
		float countScore();
		float countSP(int newNode);

		float countDistance();

		void two_opt();
		void setNodeInMiddle(int pos, int node);
		int buildT(std::vector<int>& unused_nodes, bool* used);
		void buildUnused(std::vector<int>& unused_nodes, bool* used);
		void pathTightening(std::vector<int>& unused_nodes, bool* used);
	};

	void two_param_iterative_gop(int par_i, int par_t, int distance_budget, std::vector<double>& topic_param, const NodeSet& nodes, const EdgeSet& edges, int start, int end, scorefunc_t scorefunc, spfunc_t spfunc, std::list<int>& result);
}

#endif