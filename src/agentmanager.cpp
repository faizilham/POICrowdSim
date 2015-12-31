#include "agentmanager.h"
#include <random>
#include <algorithm>
#include "helper.h"

namespace POICS {

	static std::random_device rd;
	static std::mt19937 am_rng(rd());

	Agent::Agent(int _id, Profile& profile, double entryTime, int num_topic)
	: id(_id), topic_interest(num_topic), nextUpdate(entryTime), state(AgentState::INIT){

		std::uniform_int_distribution<int> rnd_duration(profile.min_duration, profile.max_duration);
		std::uniform_real_distribution<double> rnd_topic(0.0, 1.0);

		duration = rnd_duration(am_rng);
		profile_name = profile.name;

		std::fill_n(topic_interest.begin(), num_topic, 0.0);

		auto& interestRange = profile.getInterestRanges();

		for (auto itr = interestRange.begin(); itr != interestRange.end(); ++itr){
			int idx = itr->first;
			auto range = itr->second;
			double min = range.first, max = range.second;

			topic_interest[idx] = rnd_topic(am_rng) * (max - min) + min;
		}
	}

	AgentState Agent::nextState(){
		switch(state){
			case AgentState::INIT: state = AgentState::TO_POI; break;
			case AgentState::TO_POI: state = POIPlan.empty() ? AgentState::EXITING : AgentState::TO_POI; break;
			case AgentState::IN_POI: state = AgentState::IN_POI; break;
			default: break;
		}

		return state;
	}

	int AgentSimulator::addProfile (std::string _name, double _dist){
		profiles.emplace_back(_name, _dist);
		return profiles.size() - 1;
	}

	void AgentSimulator::addInterestRange(int profile_id, std::string topic_name, double _min, double _max){
		auto topic = topic_ids.find(topic_name);

		if (topic == topic_ids.end()) except("Topic not found: " + topic_name);

		profiles[profile_id].addInterestRange(topic->second, _min, _max);
	}

	int getRandomId(std::vector<double>& distribution, double randomNum){
		double sum = 0; int i = 0;
		for (double d : distribution){
			sum += d;
			if (randomNum < sum) return i;
			++i;
		}

		return -1;
	}

	void AgentSimulator::initialize(){
		generateAgents();
		currentTimestep = 0.0;
	}

	void AgentSimulator::update(double timestep){
		currentTimestep += timestep;

		for (auto itr = activeAgents.begin(); itr != activeAgents.end(); ++itr){
		 	AgentPtr& agent = *itr;

		 	// update RVO

		 	switch(agent->state){
		 		case AgentState::TO_POI: {

		 		}
		 		break;
		 		default: break;
		 	}

			// 	if (currentTimestep > agent->entryTimestep) continue;


		}

		for (auto itr = initialAgents.begin(); itr != initialAgents.end(); ++itr){
		 	AgentPtr& agent = *itr;

			// 	if (currentTimestep > agent->entryTimestep) continue;


		}
	}

	void AgentSimulator::generateAgents(){
		std::vector<double> distribution(profiles.size());

		double timesteps = 1000; // TODO get from MapArea
		int num_topic = topic_ids.size();

		std::uniform_real_distribution<double> rnd(0.0, 1.0);

		for (int i = 0; i < num_agent; ++i){
			// select profile
			int profile_id = getRandomId(distribution, rnd(am_rng));
			double entryTime = rnd(am_rng) * timesteps;

			Agent* agent = new Agent(i, profiles[profile_id], entryTime, num_topic);

			initialAgents.push_back(AgentPtr(agent));
		}
	}
}