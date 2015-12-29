#ifndef GRAPH_H
#define GRAPH_H

#include <cstdlib>

namespace POICS{
	class NodeSet{
	private:
		double *scores = NULL; // scores[node_id][score_elmt_id]
	public:
		int num_nodes;
		int num_score_elmts;
		NodeSet(){}
		NodeSet(int _num_nodes, int _num_score_elmts);
		~NodeSet();

		void init(int _num_nodes, int _num_score_elmts);
		double getScoreElement(int node_id, int score_elmt_id) const;
		double getScore(int node_id) const;
		void setScore(int node_id, int score_elmt_id, double score);
	};

	class EdgeSet{
	private:
		int num_nodes;
		double* edges = NULL;
	public:
		EdgeSet(){}
		EdgeSet(int _num_nodes);
		~EdgeSet();

		void init(int _num_nodes);
		void addEdge(int node1, int node2, double length);
		void addEdgeSymmetric(int node1, int node2, double length);
		double getLength(int node1, int node2) const;
	};
}

#endif