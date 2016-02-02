#ifndef RNG_H
#define RNG_H

#include <random>
#include <vector>

namespace POICS {
	extern std::mt19937 am_rng;
	extern std::mt19937 gop_rng;
	extern std::mt19937 cm_rng;
	extern std::mt19937 sp_rng;
	extern std::mt19937 sim_rng;

	class RNG{
	public:
		static void setRandomSeed(unsigned int n);
		static unsigned int getRandomSeed();
	};
}

#endif