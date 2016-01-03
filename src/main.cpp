#include "xmlreader.h"
#include "compiledmap.h"
#include "simulator.h"
#include <iostream>
#include <iomanip>
#include <exception>
#include <cstdlib>
#include "imagehelper.h"

#include <memory>

using namespace POICS;
using namespace std;
int main(){

	try{
		cout << setprecision(5);
		std::unique_ptr<XMLMapReader> xm(XMLMapReader::create("example/mapfile.xml"));

		MapArea m;
		xm->build(m);

		m.timesteps = 10000;

		std::unique_ptr<XMLAgentReader> xa(XMLAgentReader::create("example/agentfile.xml"));
		AgentBuilder as(m.getTopicIds());
		xa->build(as);

		//cout << m;

		PathFinder pf;
		HMNavMesh hm(pf);
		hm.build(m);

		std::unique_ptr<Painter> painter(Painter::create(m.width, m.height, 3));

		painter->setColor(255, 0, 0);
		for (SpawnPoint& spawn : m.getSpawns()){
			painter->drawRect(spawn.border);
		}

		painter->setColor(0, 0, 255);
		for (ExitPoint& ex : m.getExits()){
			painter->drawRect(ex.border);
		}

		painter->setColor(0, 255, 0);
		for (POI& poi : m.getPois()){
			painter->drawRect(poi.border);
		}

		painter->setColor(150, 150, 150);
		for (Polygon& pl : hm.getCorridors()){
			painter->drawPoly(pl);
		}



		PlanManager pm(m, hm);

		std::unique_ptr<Simulator> sim(Simulator::create(m, as, pm));

		sim->initialize(2);

		while (!sim->finished()){
			double timestep = sim->getTimestep();
			cout<<timestep<<endl;
			for (Agent *agent : sim->getActiveAgents()){
				cout<<agent->id<<" "<<(int)(agent->state)<<"/"<<agent->position<<endl;

				if (agent->state == AgentState::INIT){
					painter->setColor(255, 150, 0);
					cout<<"route:";
					Point prev = agent->position;
					for (Point& point : agent->route){
						cout<<" "<<point;

						
						painter->drawLine(prev, point);

						prev = point;
					}
					cout<<endl;
				}

				painter->setColor(255, 150, 150);
				painter->drawPoint(agent->position);
			}
			sim->update();
		}

		painter->save("tmp/tes.bmp");

		/*AgentPtr& agent = sim->initialAgents.front();

		pm.buildPlan(agent->duration * 60 * 5.0, agent->topicInterest, agent->plan);

		cout<<pm.poiNodeIdStart<<" "<<agent->plan.size()<<endl;
*/
	}catch (const exception& e){
		cerr<<e.what();
	}

}