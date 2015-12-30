#ifndef AGENTMANAGER_H
#define AGENTMANAGER_H

#include <vector>
#include <map>

namespace POICS{
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
		std::vector<double> topic_interest;
		int duration;
		std::string profile_name;

		Agent(Profile& profile, int num_topic);
		~Agent(){}

	};


	class AgentManager{
	public:
		std::vector<Profile> profiles;
		std::map<std::string, int> topic_ids;

		AgentManager();
		~AgentManager();

		Profile& addProfile (std::string _name, double _dist);
	};
}


#endif