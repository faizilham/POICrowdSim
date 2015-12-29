#ifndef GOP_DLL_H
#define GOP_DLL_H

#include <vector>
#include <functional>

#ifdef BUILD_GOP_DLL
#define GOP2PIADLL_API __declspec(dllexport)
#else
#define GOP2PIADLL_API __declspec(dllimport)
#endif

namespace GOP {
	typedef GOP2PIADLL_API std::function<int(const NodeSet&, const std::vector<int>&)> scorefunc_t;
	typedef GOP2PIADLL_API std::function<int(const NodeSet&, const std::vector<int>&,int)> spfunc_t;

	struct GOP2PIADLL_API ScoreFunction{
		std::function<float(const std::vector<int>&)> score; // NodeSet N, vector<int> path -> float score
		std::function<float(const std::vector<int>&,int)> sp; // NodeSet N, vector<int> path, int newNode -> float projectedScore
	};

	class GOP2PIADLL_API Solution{
	public:
		float score, distance;
		int distance_budget;
		NodeSet *nodes;
		EdgeSet *edges;
		std::vector<int> path;
		scorefunc_t scorefunc;
		spfunc_t spfunc;

		Solution(int budget, NodeSet* _nodes, EdgeSet* _edges, scorefunc_t _scorefunc, spfunc_t _spfunc);
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

	Solution GOP2PIADLL_API two_param_iterative_gop(int par_i, int par_t, int distance_budget, NodeSet& nodes, EdgeSet& edges, int start, int end, scorefunc_t scorefunc, spfunc_t spfunc);
}

#endif