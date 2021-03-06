#ifndef MAPOBJECT_H
#define MAPOBJECT_H

#include "dllmacro.h"
#include "shapes.h"
#include <map>
#include <vector>
#include <iostream>

namespace POICS {
	class POICS_API POI {
	public:
		int id;
		std::string name;
		int activityType;
		int activityTime;
		Rect border;
		std::vector<double> topic_relevance;

		POI(int _id, std::string _name, int num_topic, int _activityType, int _activityTime, double x, double y, double w, double h);
		POI(int _id, std::string _name, int num_topic, int _activityType, int _activityTime, Rect& _border);

		friend std::ostream& operator<<(std::ostream& os, const POI& p);
	};

	class POICS_API SpawnPoint {
	public:
		int id; double dist;
		Rect border;

		SpawnPoint(int _id, double _dist, double x, double y, double w, double h)
		: id(_id), dist(_dist), border(x,y,w,h){}

		SpawnPoint(int _id, double _dist, Rect& _border)
		: id(_id), dist(_dist), border(_border){}

		friend std::ostream& operator<<(std::ostream& os, const SpawnPoint& sp);
	};

	class POICS_API ExitPoint {
	public:
		int id; double dist;
		Rect border;

		ExitPoint(int _id, double _dist, double x, double y, double w, double h)
		: id(_id), dist(_dist), border(x,y,w,h){}

		ExitPoint(int _id, double _dist, Rect& _border)
		: id(_id), dist(_dist), border(_border){}

		friend std::ostream& operator<<(std::ostream& os, const ExitPoint& ep);
	};

	class POICS_API MapArea{
	private:
		std::vector<POI> pois;
		std::vector<SpawnPoint> spawns;
		std::vector<ExitPoint> exits;
		std::vector<Polygon> obstacles;
		std::map<std::string, int> topic_ids;
	public:
		double agentPathWidth = 3.0; // actual agent width + some margin. TODO maybe dynamic setting
		
		double width, height;
		int startTime, endTime;
		
		MapArea(){}
		MapArea(double w, double h): width(w), height(h){}
		~MapArea(){}

		void addTopic(std::string name);

		std::map<std::string, int>& getTopicIds(){ return topic_ids;};

		int addPOI(std::string name, int activityType, int activityTime, double x, double y, double w, double h);
		int addPOI(std::string name, int activityType, int activityTime, Rect& r);
		void setTopicRelevance(int poiId, std::string topic_name, double relevance);

		int addSpawnPoint(double dist, double x, double y, double w, double h);
		int addSpawnPoint(double dist, Rect& r);

		
		int addExitPoint(double dist, double x, double y, double w, double h);
		int addExitPoint(double dist, Rect& r);

		int createObstacle();
		int addObstacle(Polygon& polygon);
		void addObstaclePoint(int id, double x, double y);

		std::vector<Polygon>& getObstacles() { return obstacles;}
		std::vector<POI>& getPois() {return pois;}
		std::vector<SpawnPoint>& getSpawns() {return spawns;}
		std::vector<ExitPoint>& getExits() {return exits;}
		std::map<std::string, int>& getTopics() {return topic_ids;}

		friend std::ostream& operator<<(std::ostream& os, const MapArea& m);
		void loadFromXML(const char* filename);
	};
}

#endif