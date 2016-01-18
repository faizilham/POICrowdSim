#include "xmlreader.h"
#include "navmesh.h"
#include "planmanager.h"
#include "simulator.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <exception>
#include <cstdlib>

#include <memory>

#include <SFML/Graphics.hpp>

using namespace POICS;
using namespace std;

static double scale = 2.5;
static double dx, dy;

void toSFVertex (Point& in, sf::Vertex& out){
	out.position = sf::Vector2f(dx + in.x * scale, dy + in.y * scale);
}

void toSFConvex(Polygon& in, sf::ConvexShape& out){
	std::vector<Point>& points = in.getPoints();
	int n = points.size();
	out.setPointCount(n);


	for (int i = 0; i < n; ++i){
		Point& p = points[i];
		out.setPoint(i, sf::Vector2f(dx + p.x * scale, dy + p.y * scale));
	}
}

void toSFRect(Rect& in, sf::RectangleShape& out){
	out.setSize(sf::Vector2f(in.w() * scale, in.h() * scale));
	out.setPosition(dx + in.x() * scale, dy + in.y() * scale);
}

void drawPoly(sf::RenderWindow& window, Polygon& poly, sf::Color color){
	Point prev = poly.getPoints().back();
	for (Point& point : poly.getPoints()){
		sf::Vertex line[2];

		toSFVertex(prev, line[0]); toSFVertex(point, line[1]);

		line[0].color = color;
		line[1].color = color;
		
		window.draw(line, 2, sf::Lines);
		prev = point;
	}
}

int main(){
	try{
		cout << setprecision(5);
		bool showroute = true;
		bool shownavmesh = true;

		std::unique_ptr<XMLMapReader> xm(XMLMapReader::create("example/mapfile.xml"));
		XMLMapReader::MAP_SCALE = 1.0;
		MapArea m;
		xm->build(m);

		int windowWidth = 800, windowHeight = 600;

		if (m.width > m.height){
			scale = windowWidth / m.width; 
			dx = 0;
			dy = (windowHeight - (m.height * scale)) / 2;
		} else {
			scale = windowHeight / m.height; 
			dy = 0;
			dx = (windowWidth - (m.width * scale)) / 2;
		}

		m.agentPathWidth = 3.0;
		Simulator::AGENT_RADIUS = 1.0;
		Simulator::AGENT_GOAL_SQUARE = 2.0; // 2.5 * 2.5
		Simulator::AGENT_MAXSPEED = 0.8;
		Simulator::AGENT_TIMEHORIZON = 2.0;
		Simulator::AGENT_TIMEHORIZONOBS = 1.0;
		Simulator::AGENT_NEIGHBORDIST = 15.0;
		
		m.timesteps = 10000;

		std::unique_ptr<XMLAgentReader> xa(XMLAgentReader::create("example/agentfile.xml"));
		AgentBuilder as(m.getTopicIds());
		xa->build(as);

		//cout << m;
		std::vector<Profile> profiles = as.getProfiles();
		std::map<std::string, sf::Color> profileColor;

		for (Profile& profile : profiles){
			int r, g, b;
			try{
				std::string colorstr = profile.extras.at("color"); int color;

				std::stringstream ss;
				ss << std::hex << colorstr;
				ss >> color;

				r = (color >> 16) & 0xFF;
				g = (color >> 8) & 0xFF;
				b = color & 0xFF;
			}catch(...){
				r = 255; g = 0; b = 0;
			}

			profileColor.insert(std::make_pair(profile.name, sf::Color(r,g,b)));
		}


		PathFinder pf;
		HMNavMesh hm(pf);
		hm.build(m);

		PlanManager pm(m, hm);
		std::unique_ptr<Simulator> sim(Simulator::create(m, as, pm));

		sim->initialize(1);

		sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "POICrowdSim");

		Rect area(0, 0, m.width, m.height);
		sf::RectangleShape rectArea;
		toSFRect(area, rectArea); rectArea.setFillColor(sf::Color::White);

		while (window.isOpen())	{
			sf::Event event;
			while (window.pollEvent(event)) {		
				if (event.type == sf::Event::Closed)
				window.close();
			}

			window.clear(sf::Color::Black);
			window.draw(rectArea);

			// draw obstacle
			for (Polygon& poly : m.getObstacles()){
				drawPoly(window, poly, sf::Color::Black);
			}

			// draw navmesh

				if (shownavmesh){
				for (Polygon& poly : hm.getCorridors()){
					sf::ConvexShape cv;
					toSFConvex(poly, cv);
					//cv.setFillColor(sf::Color::White);

					cv.setOutlineThickness(1);
					cv.setOutlineColor(sf::Color(200, 200, 200));

					window.draw(cv);
				}
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
				circ.setFillColor(profileColor[agent->profile_name]);
				circ.setPosition(dx + (agent->position.x - Simulator::AGENT_RADIUS) * scale, dy + (agent->position.y - Simulator::AGENT_RADIUS) * scale);
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