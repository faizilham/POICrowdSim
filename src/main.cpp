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

		m.timesteps = 10000;

		XMLAgentReader xa("example/agentfile.xml");
		AgentBuilder as(m.getTopicIds());
		xa >> as;

		//cout << m;

		PathFinder pf;
		HMNavMesh hm(pf);
		hm.build(m);

		PlanManager pm(m, hm);

		Simulator sim(m, as, pm);

		sim.initialize(2);

		while (!sim.finished()){
			double timestep = sim.getTimestep();
			cout<<timestep<<endl;
			for (Agent *agent : sim.getActiveAgents()){
				cout<<agent->id<<" "<<(int)(agent->state)<<"/"<<agent->position<<endl;

				if (agent->state == AgentState::INIT){
					cout<<"route:";
					for (Point& point : agent->route){
						cout<<" "<<point;
					}
					cout<<endl;
				}
			}
			sim.update();
		}

		/*AgentPtr& agent = sim.initialAgents.front();

		pm.buildPlan(agent->duration * 60 * 5.0, agent->topicInterest, agent->plan);

		cout<<pm.poiNodeIdStart<<" "<<agent->plan.size()<<endl;
*/
	}catch (const exception& e){
		cerr<<e.what();
	}

}