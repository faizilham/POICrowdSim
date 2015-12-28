#include "mapreader.h"
#include  <iomanip>
using namespace POICS;
int main(){
	std::cout << std::setprecision(5);
	XMLMapReader xm("example/mapfile.xml");
	MapArea m;
	xm >> m;
	std::cout << m;

}