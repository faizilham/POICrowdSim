#ifndef AGENTMANAGER_H
#define AGENTMANAGER_H

#include <vector>
#include <map>
#include "shapes.h"
#include "mapobject.h"
#include <memory>

namespace POICS{

	enum class AgentState {INIT, TO_POI, IN_POI, EXITING};

	class Profile{
	public:
		std::string name;
		double dist;
		int min_duration, max_duration;
		std::map<int, std::pair<double, double>> topic_interest_range;

		Profile(std::string _name, double _dist)
		: name(_name), dist(_dist){}

		~Profile(){}

		void setDuration(int _min, int _max){min_duration = _min; max_duration = _max;}

		void addInterestRange(int topic_id, double _min, double _max){
			topic_interest_range.insert(std::make_pair(topic_id, std::make_pair(_min, _max)));
		}

		std::map<int, std::pair<double, double>>& getInterestRanges(){
			return topic_interest_range;
		}
	};

	class Agent{
	public:
		

		int id;
		std::vector<double> topic_interest;
		std::vector<int> POIPlan;
		std::vector<Point> current_route;

		double nextUpdate;
		int duration;
		std::string profile_name;
		AgentState state;

		Agent(int _id, Profile& profile, double entryTime, int num_topic);
		~Agent(){}

		AgentState nextState();

	};

	typedef std::unique_ptr<Agent> AgentPtr;

	class AgentSimulator{
	private:
		void generateAgents();
	public:
		std::vector<Profile> profiles;
		std::map<std::string, int> topic_ids;
		std::vector<AgentPtr> initialAgents;
		std::vector<AgentPtr> activeAgents;
		std::vector<AgentPtr> exitAgents;

		MapArea* maparea;
		int num_agent;

		double currentTimestep;

		/* building phase */
		AgentSimulator(MapArea& _maparea): maparea(&_maparea){}
		~AgentSimulator(){}

		void setTopicIds(std::map<std::string, int>& _topic_ids){
			topic_ids = _topic_ids;
		}
		void setNumAgents(int num){
			num_agent = num;
			initialAgents.reserve(num); activeAgents.reserve(num); exitAgents.reserve(num);
		}
		int addProfile (std::string _name, double _dist);
		void addInterestRange(int profile_id, std::string topic_name, double _min, double _max);

		void initialize();

		void update(double timestep);

		void generatePlan();


	};
}


#endif