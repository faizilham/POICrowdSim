#include "helper.h"
#include "mapobject.h"
#include <algorithm>

#include "xmlreader.h"
#include <memory>

using std::endl;

namespace POICS{

	POI::POI(int _id, std::string _name, int num_topic, int _activityType, int _activityTime, double x, double y, double w, double h)
	: id(_id), name(_name), activityType(_activityType), activityTime(_activityTime), border(x,y,w,h), topic_relevance(num_topic, 0.0) {
		std::fill_n(topic_relevance.begin(), num_topic, 0.0);
	}

	POI::POI(int _id, std::string _name, int num_topic, int _activityType, int _activityTime, Rect& _border)
	: id(_id), name(_name), activityType(_activityType), activityTime(_activityTime), border(_border), topic_relevance(num_topic, 0.0) {
		std::fill_n(topic_relevance.begin(), num_topic, 0.0);
	}


	void MapArea::addTopic(std::string name){
		topic_ids.insert(make_pair(name, topic_ids.size()));
	}

	int MapArea::addPOI(std::string name, int activityType, int activityTime, double x, double y, double w, double h){
		int id = pois.size();
		pois.push_back(POI (id, name, topic_ids.size(), activityType, activityTime, x, y, w, h));
		return id;
	}

	int MapArea::addPOI(std::string name, int activityType, int activityTime, Rect& r){
		int id = pois.size();
		pois.push_back(POI (id, name, topic_ids.size(), activityType, activityTime, r));
		return id;
	}

	void MapArea::setTopicRelevance(int poiId, std::string topic_name, double relevance){
		auto topic = topic_ids.find(topic_name);

		if (topic == topic_ids.end()) except("Topic not found: " + topic_name);

		pois[poiId].topic_relevance[topic->second] = relevance;
	}

	int MapArea::addSpawnPoint(double dist, double x, double y, double w, double h){
		int id = spawns.size();
		spawns.push_back(SpawnPoint (id, dist, x, y, w, h));
		return id;
	}

	int MapArea::addSpawnPoint(double dist, Rect& r){
		int id = spawns.size();
		spawns.push_back(SpawnPoint (id, dist, r));
		return id;
	}

	int MapArea::addExitPoint(double x, double y, double w, double h){
		int id = exits.size();
		exits.push_back(ExitPoint (id, x, y, w, h));
		return id;
	}

	int MapArea::addExitPoint(Rect& r){
		int id = exits.size();
		exits.push_back(ExitPoint (id, r));
		return id;
	}

	int MapArea::createObstacle(){
		int id = obstacles.size();
		obstacles.push_back(Polygon(id));
		return id;
	}

	int MapArea::addObstacle(Polygon& polygon){
		int id = obstacles.size();
		polygon.id = id;
		obstacles.push_back(polygon);
		return id;
	}

	void MapArea::addObstaclePoint(int id, double x, double y){
		obstacles[id].addPoint(x,y);
	}

	void MapArea::loadFromXML(const char* filename){
		std::unique_ptr<XMLMapReader> xm(XMLMapReader::create(filename));
		xm->build(*this);
	}

	std::ostream& operator<<(std::ostream& os, const POI& p){
		os<<p.name<<" "<<p.activityType<<" "<<p.activityTime<<" "<<p.border<<endl;
		for (auto itr = p.topic_relevance.begin(); itr != p.topic_relevance.end(); ++itr){
			os<<*itr<<" ";
		}
		return os<<endl;

	}
	std::ostream& operator<<(std::ostream& os, const SpawnPoint& sp){
		return os<<sp.border<<" : "<<sp.dist;
	}

	std::ostream& operator<<(std::ostream& os, const ExitPoint& ep){
		return os<<ep.border;
	}

	std::ostream& operator<<(std::ostream& os, const MapArea& m){
		os<<m.width<<" "<<m.height<<endl;

		os << "topic" << endl;
		for (auto itr = m.topic_ids.begin(); itr != m.topic_ids.end(); ++itr){
			os << itr->first << ": "<< itr->second << endl;
		}

		os << "\nspawns" << endl;
		for (auto itr = m.spawns.begin(); itr != m.spawns.end(); ++itr){
			os << *itr << endl;
		}

		os << "\nexits" << endl;
		for (auto itr = m.exits.begin(); itr != m.exits.end(); ++itr){
			os << *itr << endl;
		}


		os << "\nPOI" << endl;
		for (auto itr = m.pois.begin(); itr != m.pois.end(); ++itr){
			os << *itr;
		}

		os << "\nobstacle" <<endl;
		for (const Polygon& p : m.obstacles){
			os << p << std::endl;;	
		}
		
		return os;
	}
}