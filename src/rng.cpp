#include "rng.h"
#include <random>

namespace POICS{
	static unsigned int seed;
	static std::random_device rd;
	static bool noseed = true;

	std::mt19937 am_rng(rd());
	std::mt19937 gop_rng(rd());
	std::mt19937 cm_rng(rd());
	std::mt19937 sp_rng(rd());
	std::mt19937 sim_rng(rd());

	void RNG::setRandomSeed(unsigned int n) {
		seed = n;
		noseed = false;

		am_rng.seed(n);
		gop_rng.seed(n);
		cm_rng.seed(n);
		sp_rng.seed(n);
		sim_rng.seed(n);
	}

	unsigned int RNG::getRandomSeed() {
		return noseed ? rd() : seed;
	}
}
