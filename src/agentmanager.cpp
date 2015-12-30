#include "agentmanager.h"
#include <random>
#include <algorithm>

namespace POICS {

	Agent::Agent(Profile& profile, int num_topic): topic_interest(num_topic){
		std::random_device rd;
	    std::mt19937 generator(rd());
	    
	    std::uniform_int_distribution<int> rnd_duration(profile.min_duration, profile.max_duration);
	    std::uniform_real_distribution<double> rnd_topic(0.0, 1.0);

	    duration = rnd_duration(generator);
	    profile_name = profile.name;

	    std::fill_n(topic_interest.begin(), num_topic, 0.0);

	    auto& interestRange = profile.getInterestRanges();

	    for (auto itr = interestRange.begin(); itr != interestRange.end(); ++itr){
	    	int idx = itr->first;
	    	auto range = itr->second;
	    	double min = range.first, max = range.second;

	    	topic_interest[idx] = rnd_topic(generator) * (max - min) + min;
	    }
	}

	Profile& AgentManager::addProfile (std::string _name, double _dist){
		profiles.emplace_back(_name, _dist);
		return profiles.back();
	}
}