#ifndef AGENTMANAGER_H
#define AGENTMANAGER_H

#include "dllmacro.h"
#include <vector>
#include <map>
#include "shapes.h"
#include "mapobject.h"
#include "helper.h"
#include <list>

namespace POICS{

	enum class AgentState {INIT, TO_POI, IN_POI, EXITING};

	class POICS_API Profile{
	public:
		std::string name;
		double dist;
		int min_duration, max_duration;
		std::map<int, std::pair<double, double>> topicInterestRange;
		std::map<std::string, std::string> extras;

		Profile(std::string _name, double _dist);
		~Profile();

		void setDuration(int _min, int _max);

		void addInterestRange(int topic_id, double _min, double _max);

		std::map<int, std::pair<double, double>>& getInterestRanges();
	};

	class POICS_API Agent{
	public:
		int id;
		std::vector<double> topicInterest;
		std::list<int> plan; // POI plan
		std::list<Point> route; // route, list of points

		Point position, identityPosition;
		int currentNode = -1;
		int duration;
		double nextUpdate;
		
		double startTime = 0, endTime = 0, totalVelocity = 0, walkingTimesteps = 0, poiTimesteps = 0;
		int totalpoi = 0;
		
		SolutionMeta metasolution;

		std::string profile_name;
		AgentState state;

		Agent(int _id, Profile& profile, double entryTime, int num_topic);
		~Agent(){}

		AgentState nextState();

	};

	typedef POICS_API std::list<Agent*> AgentList;

	class POICS_API AgentBuilder{
	private:
		int num_agent;
		std::vector<Profile> profiles;
		std::map<std::string, int>* topic_ids;
	public:
		double entryTime;
		/* building phase */
		AgentBuilder(std::map<std::string, int>& _topic_ids): topic_ids(&_topic_ids){}
		~AgentBuilder(){}

		void setNumAgents(int num);
		int getNumAgents() const;

		int addProfile (std::string _name, double _dist);
		void setProfileDuration(int id, int _min, int _max);
		void addProfileExtras(int id, std::string key, std::string value);
		void addInterestRange(int profile_id, std::string topic_name, double _min, double _max);

		void generateAgents(AgentList& result_agents);
		std::vector<Profile>& getProfiles(){ return profiles;}

		void loadFromXML(const char* filename);
	};
}


#endif