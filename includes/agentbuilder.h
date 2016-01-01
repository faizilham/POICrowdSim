#ifndef AGENTMANAGER_H
#define AGENTMANAGER_H

#include <vector>
#include <map>
#include "shapes.h"
#include "mapobject.h"
#include <list>

namespace POICS{

	enum class AgentState {INIT, TO_POI, IN_POI, EXITING};

	class Profile{
	public:
		std::string name;
		double dist;
		int min_duration, max_duration;
		std::map<int, std::pair<double, double>> topicInterestRange;

		Profile(std::string _name, double _dist)
		: name(_name), dist(_dist){}

		~Profile(){}

		void setDuration(int _min, int _max){min_duration = _min; max_duration = _max;}

		void addInterestRange(int topic_id, double _min, double _max){
			topicInterestRange.insert(std::make_pair(topic_id, std::make_pair(_min, _max)));
		}

		std::map<int, std::pair<double, double>>& getInterestRanges(){
			return topicInterestRange;
		}
	};

	class Agent{
	public:
		int id;
		std::vector<double> topicInterest;
		std::list<int> plan; // POI plan
		std::list<Point> route; // route, list of points

		Point position;
		int currentNode = -1;
		int duration;
		double nextUpdate;
		
		std::string profile_name;
		AgentState state;

		Agent(int _id, Profile& profile, double entryTime, int num_topic);
		~Agent(){}

		AgentState nextState();

	};

	typedef std::list<Agent*> AgentList;

	class AgentBuilder{
	private:
		int num_agent;
		std::vector<Profile> profiles;
		std::map<std::string, int>* topic_ids;
	public:
		/* building phase */
		AgentBuilder(std::map<std::string, int>& _topic_ids): topic_ids(&_topic_ids){}
		~AgentBuilder(){}

		void setNumAgents(int num){	num_agent = num;}
		int getNumAgents() const { return num_agent;}

		int addProfile (std::string _name, double _dist);
		void setProfileDuration(int id, int _min, int _max);
		void addInterestRange(int profile_id, std::string topic_name, double _min, double _max);

		void generateAgents(double totalTimesteps, AgentList& result_agents);
	};
}


#endif