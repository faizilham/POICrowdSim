#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include "shapes.h"
#include <map>
#include <vector>

namespace POICS {
	class POI {
	public:
		int id;
		std::string name;
		int activityType;
		int activityTime;
		Rect border;
		std::vector<double> topic_relevance;

		POI(int _id, std::string _name, int _activityType, int _activityTime, double x, double y, double w, double h)
		: id(_id), name(_name), activityType(_activityType), activityTime(_activityTime), border(x,y,w,h){}

		POI(int _id, std::string _name, int _activityType, int _activityTime, Rect _border)
		: id(_id), name(_name), activityType(_activityType), activityTime(_activityTime), border(_border){}
	};

	class SpawnPoint {
	public:
		int id; double dist;
		Rect border;

		SpawnPoint(int _id, double _dist, double x, double y, double w, double h)
		: id(_id), dist(_dist), border(x,y,w,h){}

		SpawnPoint(int _id, double _dist,Rect _border)
		: id(_id), dist(_dist), border(_border){}
	};

	class ExitPoint {
	public:
		int id;
		Rect border;

		ExitPoint(int _id, double x, double y, double w, double h)
		: id(_id), border(x,y,w,h){}

		ExitPoint(int _id, Rect _border)
		: id(_id), border(_border){}
	};

	class MapArea{
	public:
		std::vector<POI> pois;
		std::vector<SpawnPoint> spawns;
		std::vector<ExitPoint> exits;
		std::vector<Polygon> obstacles;
		double width, height;
		std::map<std::string, int> topic_ids;

		MapArea(){}
		MapArea(double w, double h): width(w), height(h){}
		~MapArea(){}

		void addTopic(std::string name);

		int addPOI(std::string name, int activityType, int activityTime, double x, double y, double w, double h);
		void setTopicRelevance(int poiId, std::string topic_name, double relevance);

		int addSpawnPoint(double dist, double x, double y, double w, double h);

		int addExitPoint(double x, double y, double w, double h);

		int createObstacle();
		int addObstacle(Polygon& polygon);
		void addObstaclePoint(int id, double x, double y);
	};
}

#endif