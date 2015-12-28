#include "shapes.h"
#include <cmath>
#include <cstdlib>

double POICS::Point::squareDistanceTo(const Point& p2) const{
	double dx = x-p2.x, dy = y - p2.y;

	return dx*dx + dy*dy;
}

double POICS::Point::distanceTo(const Point& p2) const{
	return std::sqrt(squareDistanceTo(p2));
}

bool POICS::Point::operator==(const Point& p2) const{
	return (std::abs(x-p2.x) < EPSILON) && (std::abs(y-p2.y) < EPSILON);
}

void POICS::Point::setAsMiddle(const Point& p1, const Point& p2){
	x = (p1.x + p2.x) / 2; y = (p1.y + p2.y) / 2;
}


bool POICS::Polygon::testNeighborhood(const Polygon& poly, Point& result_p1, Point& result_p2) const{
	int match = 0; auto pmatch = points.end();

	for (auto p1 = points.begin(); p1 != points.end(); ++p1){
		for (auto p2 = poly.points.begin(); p2 != poly.points.end(); ++p2){
			if (*p1 == *p2){
				if (match == 0){
					pmatch = p1;
					match = 1;
				} else {
					result_p1 = *pmatch;
					result_p2 = *p1;
					return true;
				}
			}
		}
	}

	return false;
}