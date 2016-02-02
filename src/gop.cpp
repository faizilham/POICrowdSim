#include "gop.h"
#include <memory>
#include <random>
#include <algorithm> 
#include <set>
#include "rng.h"

namespace POICS{
	Solution::Solution(int budget, std::vector<double>& _topic_param, const NodeSet* _nodes, const EdgeSet* _edges, ScoreFunc& _SF)
	: score(0), distance(0), distance_budget(budget), nodes(_nodes), edges(_edges), topic_param(&_topic_param), SF(&_SF){}

	Solution::~Solution(){}
	Solution::Solution(const Solution& sol){
		nodes = sol.nodes;
		edges = sol.edges;
		topic_param = sol.topic_param;
		SF = sol.SF;
		copy(sol);
	}

	void Solution::operator= (const Solution& sol){
		copy(sol);
	}

	void Solution::copy (const Solution& sol){
		score = sol.score;
		distance = sol.distance;
		path.clear();
		for (int node : sol.path){
			path.push_back(node);
		}
	}

	double Solution::countScore(){
		return SF->scorefunc(*nodes, *topic_param, path);
	}

	double Solution::countSP(int newNode){
		return SF->spfunc(*nodes, *topic_param, path, newNode);
	}

	double Solution::countDistance(){
		// for debugging only
		double sum = 0;
		int n = path.size();
		for (int i = 0; i < n - 1; ++i){
			sum += edges->getLength(path[i], path[i+1]);
		}

		return sum;
	}

	void Solution::two_opt(){
		/* (2-opt algorithm) While there exist edges (a,b), (c,d) in S s.t. d(a,b) + d(c,d) > d(a,c) + d(b,d), 
			remove edges (a,b) and (c,d) from S and add edges (a,c) and (b,d) to S. */

		int a, b, c, d; double dab, dcd, dac, dbd; bool changed; int n = path.size();
		do {
			changed = false;
			for (int i = 0; i < n - 2; ++i){
				for (int j = i + 1; j < n - 1; ++j){
					a = path[i]; b = path[i+1];
					c = path[j]; d = path[j+1];

					dab = edges->getLength(a, b); dcd = edges->getLength(c, d);
					dac = edges->getLength(a, c); dbd = edges->getLength(b, d);

					if (dab + dcd > dac + dbd){
						distance = distance - dab - dcd + dac + dbd;

						int n = j - i; // size from b and c
						for (int k = 0; k < n / 2; ++k){ // flip
							int n1 = i + 1 + k;
							int n2 = j - k;

							std::iter_swap(path.begin() + n1, path.begin() + n2);
						}

						changed = true;
						break;
					}
				}

				if (changed) break;
			}

		} while(changed);
	}

	static const double EPSILON = 1e-6;

	void Solution::process_gop(int par_i, int par_t, int POIIdx, int start, int end){
		/*
		Input: Parameters i and t, graph G = (V,E), distance matrix d for which dab is
		the distance between vertices a and b, start node s, destination node e, distance
		limit l, and score(S), a function that returns the score of a solution S.
		*/
	    
	    std::uniform_int_distribution<> random_node(POIIdx, nodes->num_nodes - 1);

		/** INITIALIZATION PHASE **/
		score = 0.0; distance = 0.0; path.clear(); std::set<int> R, L; std::vector<bool> used(nodes->num_nodes);
		std::fill_n(used.begin(), POIIdx, true);

		int available_nodes = nodes->num_nodes - POIIdx;

		for (int i = POIIdx; i < (int) used.size(); ++i){
			double sc = countSP(i);
			if (sc > EPSILON){
				used[i] = false;
			} else {
				used[i] = true;
				available_nodes -= 1;
			}
		}

		// 2. Initialize solution S to contain the single node s
		path.push_back(start); used[start] = true;
		double last_distance = 0.0; 

		// 3. While adding node e to the end of S would not cause the length of S to exceed the distance limit l.
		while (distance + edges->getLength(path.back(), end) <= distance_budget){
			int node;
			/* (a) Randomly select i nodes (with repeats allowed), s.t. each is not in S and each is not e.
			 	Store these i nodes in set L. If all nodes except e have been added	to S, then add e to the end and return the final solution.*/
			if (available_nodes == 0){
				path.push_back(end);
				distance += edges->getLength(path.back(), end);
				score = countScore();
				return;
			} else {
				L.clear();
				for (int i = 0; i < par_i; ++i){
					do{
						node = random_node(gop_rng);
					} while((node == end) || (used[node]) || (std::find(path.begin(), path.end(), node) != path.end()));

					L.insert(node);
				}
			}

			/* (b) If z is the last vertex in S, then select b in L s.t. for all q in L, d(z,b) + d(b,e) <= d(z,q) + d(q,e). */
			auto b = L.begin(); node = *b;
			double min_dis = edges->getLength(path.back(), node) + edges->getLength(node, end); ++b;
			while (b != L.end()){
				double dis = edges->getLength(path.back(), *b) + edges->getLength(*b, end);
				if (dis < min_dis){
					min_dis = dis;
					node = *b;
				}
				++b;
			}

			/* (c) Add b to the end of S. */
			last_distance = edges->getLength(path.back(), node);
			path.push_back(node); used[node] = true;
			distance += last_distance;
			--available_nodes;
		}

		// 4. Replace the last vertex in S with e.
		used[path.back()] = false; path.pop_back(); 
		distance = distance - last_distance + edges->getLength(path.back(), end);
		path.push_back(end); used[end] = true;

		/* 5. 2-opt algorithm */
		two_opt();

		/** PATH TIGHTENING PHASE **/

		// 6. Build unused list
		std::vector<int> unused_nodes;
		buildUnused(unused_nodes, used);

		// 7-8 Path tightening
		pathTightening(unused_nodes, used);


		/** PERTURBATION AND IMPROVEMENT **/

		/* 9. Flag current solution S as the best solution discovered and set y, the number of
		iterations since the last improvement in the best solution, to be 0. */
		score = countScore();
		Solution best(*this);
		int y = 0;

		// 10. While y <= t
		while(y <= par_t){
			/* (a) Randomly select i unique nodes in S, each of which is not s or e, and store them in set R.
			*/
			std::uniform_int_distribution<> random_remove(0, path.size()-1);
			R.clear();
			for (int i = 0; i < par_i; ++i){
				int node;
				do{
					node = path[random_remove(gop_rng)];
				} while (node == start || node == end || (R.find(node) != R.end()));
				R.insert(node);
			}

			/*(b) For each a in R, let b(S,a) be the node in S before a and let a(S,a) be the
				node in S after a. Remove edges (b(S,a),a) and (a,a(S,a)) and add edge (b(S,a),a(S,a))
				*/

			int bef, aft;
			for (int node : R){
				auto pos = std::find(path.begin(), path.end(), node); // TODO optimize with memoization from above
				bef = *(pos - 1); aft = *(pos + 1);
				path.erase(pos);
				
				used[node] = false;

				distance += edges->getLength(bef, aft) - edges->getLength(bef, node) - edges->getLength(node, aft);

				unused_nodes.push_back(node);
			}
			
			/*	(c) Place the vertices not in S and not in R in a list L, such that Lm is the mth
				element of the list. Define function sp(S,k) as in Step 6. Insert the elements
				into L such that sp(S,Lm) < sp(S,Lo) implies m > o.

				note: this should be unused_nodes, so let's move to

				(d) Add the contents of R in arbitrary order to the end of L.

				note: joined to previous loop
			*/

			// (e) Repeat Steps 7 through 8 with L to complete modified path tightening.
			pathTightening(unused_nodes, used);

			/* (f) 2-opt algorithm */
			two_opt();

			// (g) Repeat Steps 6 through 8 to complete unmodified path tightening.

			buildUnused(unused_nodes, used);
			pathTightening(unused_nodes, used);

			/* (h) If score(S) is higher than the score of the best solution yet discovered, flag
			current solution S as the best solution discovered and set y = 0. Otherwise, set y = y + 1. */

			score = countScore();

			if (score > best.score){
				best.copy(*this);
				y = 0;
			} else {
				++y;	
			}
		}

	}

	void Solution::pathTightening(std::vector<int>& unused_nodes, std::vector<bool>& used){
		/* 7. Build T */
		// since T is redefined every iteration, no need to save all and only take the 1st
		int T = buildT(unused_nodes, used);
		
		// 8. While |T| > 0
		while(T != -1){
			// (a) Select Lb in T s.t. b ≤ j for all Lj in T.
			int lb = T;

			// (b) Select edge (v,w) in S s.t. dvLb +dLbw − dvw ≤ dxLb +dLby − dxy for all (x,y) in S
			int n = path.size(); int a, b; double dal, dlb, dab, dmin = 0; int imin;

			for (int i = 0; i < n - 1; ++i){
				a = path[i]; b = path[i+1];
				dab = edges->getLength(a,b); dal = edges->getLength(a,lb); dlb = edges->getLength(lb,b);
				double dnew = dal + dlb - dab;

				if (dmin > dnew || i == 0){
					dmin = dnew; imin = i + 1;
				}
			}

			// (c) Remove edge (v,w) from S and add edges (v,Lb) and (Lb,w) to S.
			path.insert(path.begin() + imin, lb);
			distance += dmin;

			// (d) Redefine T as in Step 7.
			T = buildT(unused_nodes, used);
		}
	}

	void Solution::buildUnused(std::vector<int>& unused_nodes, std::vector<bool>& used){
		/* 6. Place the vertices not in S in a list L, such that Lm is the mth element of the list.
			Define function sp(S,k) = score(T), where T is S with vertex k inserted at arbitrary location. 
			Insert the elements into L such that sp(S,Lm) < sp(S,Lo), implies m > o. */
		int max = 0, max_id = -1;

		unused_nodes.clear();

		for (int i = 0; i < nodes->num_nodes; ++i) {
			if (!used[i]) {
				unused_nodes.push_back(i);
				int sp = countSP(i);
				if (sp > max){
					max = sp; max_id = unused_nodes.size() - 1;
				}
			}
		}

		if (max_id > -1)
			std::iter_swap(unused_nodes.begin(), unused_nodes.begin() + max_id); // TODO try with sort, maybe not so efficient
	}

	int Solution::buildT(std::vector<int>& unused_nodes, std::vector<bool>& used){
		/* Define set T = {Lm in L,Lm not in S : there exist (a,b) in S : the length of S is less than budget if
			edge (a,b) is removed from S and edges (a,Lm) and (Lm,b) are added to S}.
		*/

		int n = path.size(); int a, b, lm; double dab, dal, dlb;
		for (auto it = unused_nodes.begin(); it != unused_nodes.end(); ++it){
			lm = *it;
			for (int i = 0; i < n - 1; ++i){
				a = path[i]; b = path[i+1];
				dab = edges->getLength(a,b); dal = edges->getLength(a,lm); dlb = edges->getLength(lm,b);

				if (distance - dab + dal + dlb < distance_budget){
					unused_nodes.erase(it);
					used[lm] = true;
					return lm;
				}
			}
		}

		return -1;
	}

	void two_param_iterative_gop(int par_i, int par_t, int distance_budget, std::vector<double>& topic_param, const NodeSet& nodes, const EdgeSet& edges, int POIIdx, int start, int end, ScoreFunc& SF, std::list<int>& result){
		Solution old(distance_budget, topic_param, &nodes, &edges, SF), current(distance_budget, topic_param, &nodes, &edges, SF);

		do{
			old = current;
			current.process_gop(par_i, par_t, POIIdx, start, end);
		}while (old.score < current.score);

		result.clear();
		for (int p : old.path){
			result.push_back(p);
		}
	}
}