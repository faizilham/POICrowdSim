#ifndef RNG_H
#define RNG_H

#include <random>
#include <vector>

namespace POICS {
	class RNG{
	public:
		static void setRandomSeed(unsigned int n);
		static unsigned int getRandomSeed();
		static std::mt19937& createRNG();
	};
}

#endif