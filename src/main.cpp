#include "xmlreader.h"
#include "compiledmap.h"
#include "simulator.h"
#include <iostream>
#include <iomanip>
#include <exception>
using namespace POICS;
using namespace std;
int main(){

	try{
		cout << setprecision(5);
		XMLMapReader xm("example/mapfile.xml");

		MapArea m;
		xm >> m;

		XMLAgentReader xa("example/agentfile.xml");
		AgentBuilder as(m.getTopicIds());
		xa >> as;

		//cout << m;

		PathFinder pf;
		HMNavMesh hm(pf);
		hm.build(m);

		PlanManager pm(m, hm);

		Simulator sim(m, as, pm);

		sim.initialize(0.5);

		while (!sim.finished()){
			double timestep = sim.getTimestep();
			cout<<timestep<<endl;
			for (const AgentPtr& agent : sim.getActiveAgents()){
				cout<<agent->position<<endl;
			}
			sim.update();
		}

	}catch (const exception& e){
		cerr<<e.what();
	}

}