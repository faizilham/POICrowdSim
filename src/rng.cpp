#include "rng.h"
#include <random>

namespace POICS{
	static unsigned int seed;
	static std::random_device rd;
	static bool noseed = true;
	static std::vector<std::mt19937> rngs;

	void RNG::setRandomSeed(unsigned int n) {
		seed = n;
		noseed = false;

		for (std::mt19937& rng : rngs){
			rng.seed(n);
		}
	}

	unsigned int RNG::getRandomSeed() {
		return noseed ? rd() : seed;
	}

	std::mt19937& RNG::createRNG(){
		rngs.emplace_back(getRandomSeed());

		return rngs.back();
	}
}
