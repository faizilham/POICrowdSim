#include "agentbuilder.h"
#include <random>
#include <algorithm>
#include "helper.h"

#include "xmlreader.h"
#include <memory>
#include "rng.h"

namespace POICS {
	static std::mt19937 am_rng(RNG::getRandomSeed());

	Profile::Profile(std::string _name, double _dist): name(_name), dist(_dist){}
	Profile::~Profile(){}

	void Profile::setDuration(int _min, int _max){min_duration = _min; max_duration = _max;}

	void Profile::addInterestRange(int topic_id, double _min, double _max){
		topicInterestRange.insert(std::make_pair(topic_id, std::make_pair(_min, _max)));
	}

	std::map<int, std::pair<double, double>>& Profile::getInterestRanges(){
		return topicInterestRange;
	}

	Agent::Agent(int _id, Profile& profile, double entryTime, int num_topic)
	: id(_id), topicInterest(num_topic), nextUpdate(entryTime), state(AgentState::INIT){

		std::uniform_int_distribution<int> rnd_duration(profile.min_duration, profile.max_duration);
		std::uniform_real_distribution<double> rnd_topic(0.0, 1.0);

		duration = rnd_duration(am_rng);
		profile_name = profile.name;

		std::fill_n(topicInterest.begin(), num_topic, 0.0);

		auto& interestRange = profile.getInterestRanges();

		for (auto itr = interestRange.begin(); itr != interestRange.end(); ++itr){
			int idx = itr->first;
			auto range = itr->second;
			double min = range.first, max = range.second;

			topicInterest[idx] = rnd_topic(am_rng) * (max - min) + min;
		}
	}

	AgentState Agent::nextState(){
		switch(state){
			case AgentState::INIT: state = AgentState::TO_POI; break;
			case AgentState::TO_POI: state = plan.empty() ? AgentState::EXITING : AgentState::IN_POI; break;
			case AgentState::IN_POI: state = AgentState::TO_POI; break;
			default: break;
		}

		return state;
	}

	int AgentBuilder::addProfile (std::string _name, double _dist){
		profiles.emplace_back(_name, _dist);
		return profiles.size() - 1;
	}

	void AgentBuilder::setProfileDuration(int profile_id, int _min, int _max){
		profiles[profile_id].setDuration(_min, _max);
	}

	void AgentBuilder::addProfileExtras(int id, std::string key, std::string value){
		profiles[id].extras.insert(std::make_pair(key, value));	
	}

	void AgentBuilder::addInterestRange(int profile_id, std::string topic_name, double _min, double _max){
		auto topic = topic_ids->find(topic_name);

		if (topic == topic_ids->end()) except("Topic not found: " + topic_name);

		profiles[profile_id].addInterestRange(topic->second, _min, _max);
	}

	int getRandomId(std::vector<Profile>& profiles, double randomNum){
		double sum = 0; int i = 0;
		for (Profile& p : profiles){
			sum += p.dist;
			if (randomNum < sum) return i;
			++i;
		}

		return -1;
	}

	void AgentBuilder::setNumAgents(int num){ num_agent = num;}
	int AgentBuilder::getNumAgents() const { return num_agent;}

	void AgentBuilder::generateAgents(double totalTimesteps, AgentList& result_agents){
		int num_topic = topic_ids->size();

		std::uniform_real_distribution<double> rnd(0.0, 1.0);

		for (int i = 0; i < num_agent; ++i){
			// select profile
			int profile_id = getRandomId(profiles, rnd(am_rng));
			double entryTime = rnd(am_rng) * (totalTimesteps / 2); // TODO

			Agent* agent = new Agent(i, profiles[profile_id], entryTime, num_topic);

			result_agents.push_back(agent);
		}
	}

	void AgentBuilder::loadFromXML(const char* filename){
		std::unique_ptr<XMLAgentReader> xa(XMLAgentReader::create(filename));
		xa->build(*this);
	}
}