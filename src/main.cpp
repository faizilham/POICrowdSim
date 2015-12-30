#include "mapreader.h"
#include "compiledmap.h"
#include  <iomanip>
#include <exception>
using namespace POICS;
int main(){

	try{
		std::cout << std::setprecision(5);
		XMLMapReader xm("example/mapfile.xml");
		MapArea m;
		xm >> m;
		//std::cout << m;

		PathFinder pf;
		HMNavMesh hm(pf);
		hm.build(m);

		AStarAbstractGraph(m, hm);
	}catch (const std::exception& e){
		std::cerr<<e.what();
	}

}