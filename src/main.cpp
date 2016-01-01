#include "xmlreader.h"
#include "compiledmap.h"
#include <iostream>
#include <iomanip>
#include <exception>
using namespace POICS;
int main(){

	try{
		std::cout << std::setprecision(5);
		XMLMapReader xm("example/mapfile.xml");

		MapArea m;
		xm >> m;

		XMLAgentReader xa("example/agentfile.xml");
		AgentBuilder as(m.getTopicIds());
		xa >> as;

		//std::cout << m;

		PathFinder pf;
		HMNavMesh hm(pf);
		hm.build(m);

		PlanManager pm(m, hm);
	}catch (const std::exception& e){
		std::cerr<<e.what();
	}

}