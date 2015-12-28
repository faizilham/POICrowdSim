#include <exception>
#include "mapobject.h"

void POICS::MapArea::addTopic(std::string name){
	topic_ids.insert(make_pair(name, topic_ids.size()));
}

int POICS::MapArea::addPOI(std::string name, int activityType, int activityTime, double x, double y, double w, double h){
	int id = pois.size();
	pois.push_back(POI (id, name, activityType, activityTime, x, y, w, h));
	return id;
}

void POICS::MapArea::setTopicRelevance(int poiId, std::string topic_name, double relevance){
	auto topic = topic_ids.find(topic_name);

	if (topic == topic_ids.end()) throw std::runtime_error("Topic not found: " + topic_name);

	pois[poiId].topic_relevance[topic->second] = relevance;
}

int POICS::MapArea::addSpawnPoint(double dist, double x, double y, double w, double h){
	int id = spawns.size();
	spawns.push_back(SpawnPoint (id, dist, x, y, w, h));
	return id;
}

int POICS::MapArea::addExitPoint(double x, double y, double w, double h){
	int id = exits.size();
	exits.push_back(ExitPoint (id, x, y, w, h));
	return id;
}

int POICS::MapArea::createObstacle(){
	int id = obstacles.size();
	obstacles.push_back(Polygon(id));
	return id;
}

int POICS::MapArea::addObstacle(Polygon& polygon){
	int id = obstacles.size();
	polygon.id = id;
	obstacles.push_back(polygon);
	return id;
}

void POICS::MapArea::addObstaclePoint(int id, double x, double y){
	obstacles[id].addPoint(x,y);
}