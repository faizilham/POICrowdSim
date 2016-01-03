#include "xmlreader.h"
#include "compiledmap.h"
#include "simulator.h"
#include <iostream>
#include <iomanip>
#include <exception>
#include <cstdlib>

#include <memory>

#include <SFML/Graphics.hpp>

using namespace POICS;
using namespace std;

static double scale = 2.5;
static double width;
static double height;

void toSFVertex (Point& in, sf::Vertex& out){
	out.position = sf::Vector2f(in.x * scale, in.y * scale);
}

void toSFConvex(Polygon& in, sf::ConvexShape& out){
	std::vector<Point>& points = in.getPoints();
	int n = points.size();
	out.setPointCount(n);


	for (int i = 0; i < n; ++i){
		Point& p = points[i];
		out.setPoint(i, sf::Vector2f(p.x * scale, p.y * scale));
	}
}

void toSFRect(Rect& in, sf::RectangleShape& out){
	out.setSize(sf::Vector2f(in.w() * scale, in.h() * scale));
	out.setPosition(in.x() * scale, in.y() * scale);
}

int main(){
	try{
		cout << setprecision(5);
		bool showroute = true;

		std::unique_ptr<XMLMapReader> xm(XMLMapReader::create("example/mapfile.xml"));

		MapArea m;
		scale = 3.0;
		width = (scale * m.width);
		height = (scale * m.height);

		xm->build(m);
		m.agentPathWidth = 3.0;
		Simulator::AGENT_RADIUS = 1.0;
		Simulator::AGENT_GOAL_SQUARE = 1.0; // 2.5 * 2.5
		Simulator::AGENT_MAXSPEED = 1.0;
		Simulator::AGENT_TIMEHORIZON = 5.0;
		Simulator::AGENT_TIMEHORIZONOBS = 1.0;
		Simulator::AGENT_NEIGHBORDIST = 10.0;
		
		m.timesteps = 10000;

		std::unique_ptr<XMLAgentReader> xa(XMLAgentReader::create("example/agentfile.xml"));
		AgentBuilder as(m.getTopicIds());
		xa->build(as);

		//cout << m;

		PathFinder pf;
		HMNavMesh hm(pf);
		hm.build(m);

		PlanManager pm(m, hm);
		std::unique_ptr<Simulator> sim(Simulator::create(m, as, pm));

		sim->initialize(1);

		sf::RenderWindow window(sf::VideoMode(640, 480), "POICrowdSim");

		while (window.isOpen())	{
			sf::Event event;
			while (window.pollEvent(event)) {		
				if (event.type == sf::Event::Closed)
				window.close();
			}

			window.clear(sf::Color::Black);

			// draw navmesh
			for (Polygon& poly : hm.getCorridors()){
				sf::ConvexShape cv;
				toSFConvex(poly, cv);
				cv.setFillColor(sf::Color::White);

				cv.setOutlineThickness(1);
				cv.setOutlineColor(sf::Color(160, 160, 160));

				window.draw(cv);
			}

			// draw spawns
			for (SpawnPoint& spawn : m.getSpawns()){
				sf::RectangleShape rect;
				toSFRect(spawn.border, rect);
				rect.setFillColor(sf::Color(255, 200, 200));
				window.draw(rect);
			}

			for (ExitPoint& ex : m.getExits()){
				sf::RectangleShape rect;
				toSFRect(ex.border, rect);
				rect.setFillColor(sf::Color(200, 255, 200));
				window.draw(rect);
			}

			for (POI& poi : m.getPois()){
				sf::RectangleShape rect;
				toSFRect(poi.border, rect);
				rect.setFillColor(sf::Color(200, 200, 255));
				window.draw(rect);
			}

			for (Agent *agent : sim->getActiveAgents()){
				if (showroute && !agent->route.empty()){

					Point prev = agent->position;
					for (Point& point : agent->route){
						sf::Vertex line[2];

						toSFVertex(prev, line[0]); toSFVertex(point, line[1]);

						line[0].color = sf::Color(255, 150, 0);
						line[1].color = sf::Color(255, 150, 0);
						
						window.draw(line, 2, sf::Lines);
						prev = point;
					}
				}

				sf::CircleShape circ(Simulator::AGENT_RADIUS * scale);
				circ.setFillColor(sf::Color::Red);
				circ.setPosition((agent->position.x - Simulator::AGENT_RADIUS) * scale, (agent->position.y - Simulator::AGENT_RADIUS) * scale);
				window.draw(circ);
			}


			// end the current frame
			window.display();

			// update
			if (!sim->finished()){
				sim->update();
			}

			sf::sleep(sf::milliseconds(50));
		}

		return 0;


		/*std::unique_ptr<Painter> painter(Painter::create(m.width, m.height, 3));

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

		painter->save("tmp/tes.bmp");*/

		/*AgentPtr& agent = sim->initialAgents.front();

		pm.buildPlan(agent->duration * 60 * 5.0, agent->topicInterest, agent->plan);

		cout<<pm.poiNodeIdStart<<" "<<agent->plan.size()<<endl;
*/
	}catch (const exception& e){
		cerr<<e.what();
	}

}