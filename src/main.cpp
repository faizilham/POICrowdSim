#include "mapreader.h"
#include "mapcompiler.h"
#include  <iomanip>
#include <exception>
using namespace POICS;
int main(){

	try{
		std::cout << std::setprecision(5);
		XMLMapReader xm("example/mapfile.xml");
		MapArea m;
		xm >> m;
		std::cout << m;
		
		HMNavMesh hm;
		hm.build(m);
	}catch (const std::exception& e){
		std::cerr<<e.what();
	}

}