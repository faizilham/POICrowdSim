#include "navmesh.h"
#include "planmanager.h"
#include "simulator.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <exception>
#include <cstdlib>
#include <cstring>
#include <string>
#include <memory>
#include "rng.h"
#include <SFML/Graphics.hpp>

using namespace POICS;
using namespace std;

static double scale;
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

int main(int argc, char** argv){
	try{
		sf::Font font;
		if (!font.loadFromFile("tmp/arial.ttf"))
		{
		    exit(1);
		}

		if (argc < 2) {
			cout << "Usage: poicrowd scenarioname [--route] [--navmesh] \n";
			exit(1);
		}

		cout << setprecision(5);
		bool showroute = false;
		bool shownavmesh = false;
		bool meshdetail = false;
		bool trianglenavmesh = false;
		bool makelane = true;
		CornerSmoothing smoothing = CornerSmoothing::POLYOFFSET;
		int skip = 0;
		int track = -1;
		int numagentparam = 0;

		if (argc > 2){
			for (int i = 2; i < argc; ++i){
				string arg = argv[i];

				if (arg == "--route"){
					showroute = true;
				} else if (arg == "--navmesh"){
					shownavmesh = true;
				} else if (arg == "--meshdetail"){
					shownavmesh = true;
					meshdetail = true;
				} else if (arg == "--seed") {
					if (i == argc - 1) exit(1);
					string arg2 = argv[i+1];

					unsigned int seed = (unsigned int) stoi(arg2);
					RNG::setRandomSeed(seed);
					
					i = i + 1;
				} else if (arg == "--skip") {
					if (i == argc - 1) exit(1);
					string arg2 = argv[i+1];
					skip = stoi(arg2);

					i = i + 1;
				} else if (arg == "--track") {
					if (i == argc - 1) exit(1);
					string arg2 = argv[i+1];
					track = stoi(arg2);

					i = i + 1;
				} else if (arg == "--numagent") {
					if (i == argc - 1) exit(1);
					string arg2 = argv[i+1];

					numagentparam = stoi(arg2);
					
					i = i + 1;
				} else if (arg == "--triangle") {
					trianglenavmesh = true;
				} else if (arg == "--nolane") {
					makelane = false;
				} else if (arg == "--portal") {
					smoothing = CornerSmoothing::PORTAL;
				} else if (arg == "--nosmooth") {
					smoothing = CornerSmoothing::NONE;
				} else {
					cout<<"Unknown option: "<<arg<<"\n";
				}
			}
		}

		string mapfile = string(argv[1]) + string(".xarea");
		string agentfile = string(argv[1]) + string(".xprof");

		MapArea m;
		m.loadFromXML(mapfile.c_str());

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

		/*m.agentPathWidth = 3.0;
		Simulator::AGENT_RADIUS = 1.0;
		Simulator::AGENT_GOAL_SQUARE = 1.5; // 2.5 * 2.5
		Simulator::AGENT_MAXSPEED = 1.0;
		Simulator::AGENT_TIMEHORIZON = 2.0;
		Simulator::AGENT_TIMEHORIZONOBS = 1.0;
		Simulator::AGENT_NEIGHBORDIST = 15.0;*/
		
		AgentBuilder as(m.getTopicIds());
		as.loadFromXML(agentfile.c_str());

		if (numagentparam > 0) as.setNumAgents(numagentparam);

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
		HMNavMesh hm(pf, trianglenavmesh, makelane, smoothing);
		hm.build(m);

		PlanManager pm(m, hm);
		std::unique_ptr<Simulator> sim(Simulator::create(m, as, pm));

		cout<<"Calculating Plan\n";
		sim->initialize(1);

		sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "POICrowdSim");

		Rect area(0, 0, m.width, m.height);
		sf::RectangleShape rectArea;
		toSFRect(area, rectArea); rectArea.setFillColor(sf::Color::White);

		bool paused = false;

		while (sim->getTimestep() < skip){
			sf::Event event;
			while (window.pollEvent(event)) {		
				if (event.type == sf::Event::Closed) {
					window.close();
					break;
				} 
			}

			if (!sim->finished()){
				sim->update();
			} else {
				break;
			}

			window.clear(sf::Color::Black);

			sf::Text ttext;
			ttext.setFont(font);
			string ctstep;
			stringstream ss; ss << setprecision(4); ss << sim->getTimestep(); ss >> ctstep;

			ttext.setString(ctstep);
			ttext.setCharacterSize(16); // in pixels, not points!
			// set the color
			ttext.setColor(sf::Color::Yellow);
			window.draw(ttext);

			// end the current frame
			window.display();

			paused = true;
		}

		int selectedAgentId = -1;
		bool changeSelect;

		while (window.isOpen())	{
			sf::Event event;
			while (window.pollEvent(event)) {		
				if (event.type == sf::Event::Closed) {
					window.close();
					break;
				} else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space){
					paused = !paused;
				} else if (event.type == sf::Event::MouseButtonPressed) {
					if (event.mouseButton.button == sf::Mouse::Left) {
						selectedAgentId = -1; changeSelect = true;

						for (Agent *agent : sim->getActiveAgents()){
							double ax = dx + (agent->position.x - Simulator::AGENT_RADIUS) * scale;
							double ay = dy + (agent->position.y - Simulator::AGENT_RADIUS) * scale;
							double ar = Simulator::AGENT_RADIUS * scale;
							Point apos(ax, ay), mpos(event.mouseButton.x - ar, event.mouseButton.y - ar);

							if (apos.squareDistanceTo(mpos) < ar * ar){
								selectedAgentId = agent->id;
								break;
							}
						}
					}
				}
			}

			if (paused || sim->finished()){

				if (changeSelect){
					changeSelect = false;
				} else {
					sf::sleep(sf::milliseconds(50));
					continue;
				}
			} else {
				// update
				sim->update();
			}

			window.clear(sf::Color::Black);
			window.draw(rectArea);

			// draw navmesh
			if (shownavmesh){
				for (Polygon& poly : hm.getCorridors()){
					sf::ConvexShape cv;
					toSFConvex(poly, cv);
					//cv.setFillColor(sf::Color::White);

					cv.setOutlineThickness(1);
					cv.setOutlineColor(sf::Color(200, 200, 200));
					window.draw(cv);

					if (meshdetail){
						double rad = 0.4;
						for (Portal& portal : poly.getNeighbors()){
							sf::CircleShape cc1(rad * scale), cc2(rad * scale);			
							cc1.setFillColor(sf::Color::Blue);
							cc2.setFillColor(sf::Color::Green);
							cc1.setPosition(dx + (portal.p1.x - rad) * scale, dy + (portal.p1.y - rad) * scale);
							cc2.setPosition(dx + (portal.p2.x - rad) * scale, dy + (portal.p2.y - rad) * scale);
							window.draw(cc1);
							window.draw(cc2);
						}
					}
				}
			}

			// draw obstacle
			for (Polygon& poly : m.getObstacles()){
				drawPoly(window, poly, sf::Color::Black);
			}

			// draw spawns
			for (SpawnPoint& spawn : m.getSpawns()){
				sf::RectangleShape rect;
				toSFRect(spawn.border, rect);
				rect.setFillColor(sf::Color(255, 160, 160));
				window.draw(rect);
			}

			for (ExitPoint& ex : m.getExits()){
				sf::RectangleShape rect;
				toSFRect(ex.border, rect);
				rect.setFillColor(sf::Color(160, 255, 160));
				window.draw(rect);
			}

			for (POI& poi : m.getPois()){
				sf::RectangleShape rect;
				toSFRect(poi.border, rect);
				rect.setFillColor(sf::Color(160, 160, 255));
				window.draw(rect);

				sf::Text text;
				text.setFont(font);

				// set the string to display
				text.setString(poi.name);

				float th = 12;
				text.setCharacterSize(th); // in pixels, not points!
				th = th / 2;
				Point center = poi.border.center();
				float tw = text.getLocalBounds().width / 2;

				text.setPosition(dx + center.x * scale - tw, dy + center.y * scale - th);

				// set the color
				text.setColor(sf::Color::Black);
				window.draw(text);
			}

			/*if (shownavmesh) {
				std::vector<Polygon>& corridors = hm.getCorridors();

				for (Polygon& poly : corridors){
					Point center = poly.center();
					double rad = 0.4;

					sf::CircleShape cc(rad * scale);			
					cc.setFillColor(sf::Color::Red);
					cc.setPosition(dx + (center.x - rad) * scale, dy + (center.y - rad) * scale);
					window.draw(cc);

					 for (Portal& portal : poly.getNeighbors()){
					// 	sf::Vertex line[3];
					// 	Point center3 = portal.center;
					// 	Point center2 = corridors[portal.to_id].center();

					// 	toSFVertex(center, line[0]); toSFVertex(center2, line[1]);
					// 	toSFVertex(center3, line[2]); 

					// 	line[0].color = sf::Color(255, 0, 0);
					// 	line[1].color = sf::Color(255, 0, 0);
					// 	line[2].color = sf::Color(255, 0, 0);
						
					// 	window.draw(line, 2, sf::Lines);


						sf::CircleShape cc1(rad * scale), cc2(rad * scale);			
						cc1.setFillColor(sf::Color::Blue);
						cc2.setFillColor(sf::Color::Green);
						cc1.setPosition(dx + (portal.p1.x - rad) * scale, dy + (portal.p1.y - rad) * scale);
						cc2.setPosition(dx + (portal.p2.x - rad) * scale, dy + (portal.p2.y - rad) * scale);
						window.draw(cc1);
						window.draw(cc2);
					}		

					sf::Text text;
					text.setFont(font);
					
					// set the string to display
					text.setString(to_string(poly.id));

					text.setCharacterSize(12);

					text.setPosition(dx + center.x * scale, dy + center.y * scale);

					// set the color
					text.setColor(sf::Color::Black);
					window.draw(text);
				}
			}*/

			for (Agent *agent : sim->getActiveAgents()){

				if (((showroute && (selectedAgentId == -1)) || (agent->id == selectedAgentId)) && !agent->route.empty() ){

					Point prev = agent->position;
					for (Point& point : agent->route){

						/*double rad = 0.5;

						sf::CircleShape cc(rad * scale);			
						cc.setFillColor(sf::Color::Blue);
						cc.setPosition(dx + (point.x - rad) * scale, dy + (point.y - rad) * scale);
						window.draw(cc);*/

						sf::Vertex line[2];

						toSFVertex(prev, line[0]); toSFVertex(point, line[1]);

						line[0].color = sf::Color(255, 150, 0);
						line[1].color = sf::Color(255, 150, 0);							
						
						window.draw(line, 2, sf::Lines);
						prev = point;
					}
				}

				double ax = dx + (agent->position.x - Simulator::AGENT_RADIUS) * scale;
				double ay = dy + (agent->position.y - Simulator::AGENT_RADIUS) * scale;
				double ar = Simulator::AGENT_RADIUS * scale;

				sf::CircleShape circ(ar);			
				circ.setFillColor(profileColor[agent->profile_name]);
				circ.setPosition(ax, ay);
				window.draw(circ);

				if (agent->id == track){
					sf::RectangleShape rect;
					rect.setSize(sf::Vector2f(2*ar, 2*ar));
					rect.setPosition(ax, ay);
					rect.setFillColor(sf::Color::Transparent);
					rect.setOutlineThickness(1);
					rect.setOutlineColor(sf::Color(255, 0, 0));
					window.draw(rect);
				}
			}

			sf::Text ttext;
			ttext.setFont(font);
			string ctstep;
			stringstream ss; ss << setprecision(4); ss << sim->getTimestep(); ss >> ctstep;

			ttext.setString(ctstep);
			ttext.setCharacterSize(16); // in pixels, not points!
			// set the color
			ttext.setColor(sf::Color::Yellow);
			window.draw(ttext);

			// end the current frame
			window.display();

			sf::sleep(sf::milliseconds(50));
		}

	} catch (const exception& e) {
		cerr<<e.what();
	}

	return 0;
}