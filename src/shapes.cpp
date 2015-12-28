#include "shapes.h"
#include <cmath>
#include <cstdlib>

namespace POICS{
	double Point::squareDistanceTo(const Point& p2) const{
		double dx = x-p2.x, dy = y - p2.y;

		return dx*dx + dy*dy;
	}

	double Point::distanceTo(const Point& p2) const{
		return std::sqrt(squareDistanceTo(p2));
	}

	bool Point::operator==(const Point& p2) const{
		return (std::abs(x-p2.x) < EPSILON) && (std::abs(y-p2.y) < EPSILON);
	}

	void Point::setAsMiddle(const Point& p1, const Point& p2){
		x = (p1.x + p2.x) / 2; y = (p1.y + p2.y) / 2;
	}

	void Rect::copyToPolygonCW(Polygon& pl) const{
		pl.reset();
		pl.addPoint(pos.x, pos.y);
		pl.addPoint(pos.x, pos.y + height);
		pl.addPoint(pos.x + width, pos.y + height);
		pl.addPoint(pos.x + width, pos.y);
	}

	void Rect::copyToPolygonCCW(Polygon& pl) const{
		pl.reset();
		pl.addPoint(pos.x, pos.y);
		pl.addPoint(pos.x + width, pos.y);
		pl.addPoint(pos.x + width, pos.y + height);
		pl.addPoint(pos.x, pos.y + height);
	}

	bool Polygon::testNeighborhood(const Polygon& poly, Point& result_p1, Point& result_p2) const{
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

	std::ostream& operator<<(std::ostream& os, const Point& p){
		return os<<"("<<p.x<<", "<<p.y<<")";
	}

	std::ostream& operator<<(std::ostream& os, const Rect& r){
		return os<<"("<<r.pos.x<<", "<<r.pos.y<<", "<<r.width<<", "<<r.height<<")";
	}

	std::ostream& operator<<(std::ostream& os, const Polygon& pl){
		os<<"< ";
		for (const Point& p : pl.points){
			os<<p<<" ";
		}
		return os<<">";
	}
}

