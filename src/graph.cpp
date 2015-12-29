#include "graph.h"
#include <memory>

namespace POICS{
	NodeSet::NodeSet(int _num_nodes, int _num_score_elmts): num_nodes(_num_nodes), num_score_elmts(_num_score_elmts){
		int n = _num_nodes*_num_score_elmts;
		scores = new double[n];
		std::fill_n(scores, n, 0.0);
	}

	NodeSet::~NodeSet(){
		delete[] scores;
	}

	void NodeSet::init(int _num_nodes, int _num_score_elmts){
		if (scores == NULL){
			num_nodes = _num_nodes; num_score_elmts = _num_score_elmts;

			int n = _num_nodes*_num_score_elmts;
			scores = new double[n];
			std::fill_n(scores, n, 0.0);
		}
	}

	double NodeSet::getScoreElement(int node_id, int score_elmt_id) const{
		return scores[node_id*num_score_elmts + score_elmt_id];
	}

	double NodeSet::getScore(int node_id) const{
		double sum = 0;
		for (int i = 0; i < num_score_elmts; ++i){
			sum += getScoreElement(node_id, i) / num_score_elmts;	
		}

		return sum;
	}

	void NodeSet::setScore(int node_id, int score_elmt_id, double score){
		scores[node_id*num_score_elmts + score_elmt_id] = score;
	}

	EdgeSet::EdgeSet(int _num_nodes): num_nodes(_num_nodes){
		edges = new double[_num_nodes * _num_nodes];
	}

	void EdgeSet::init(int _num_nodes){
		if (edges == NULL){
			num_nodes = _num_nodes;
			edges = new double[_num_nodes * _num_nodes];
		}
	}

	EdgeSet::~EdgeSet(){
		delete[] edges;
	}

	void EdgeSet::addEdge(int node1, int node2, double length){
		edges[node1 * num_nodes + node2] = length;
	}

	double EdgeSet::getLength(int node1, int node2) const{
		return edges[node1 * num_nodes + node2];
	}
}