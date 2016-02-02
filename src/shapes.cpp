#include "shapes.h"
#include <cmath>
#include <cstdlib>
#include <random>
#include <iostream>
#include "rng.h"

namespace POICS{
	static std::mt19937 sp_rng(RNG::getRandomSeed());

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

	void Rect::set(double x, double y, double w, double h){
		pos.x = x; pos.y = y; width = w; height = h; calcCenterPoint();
	}

	void Rect::setPos(double x, double y){
		pos.x = x; pos.y = y; calcCenterPoint();
	}

	void Rect::calcCenterPoint(){
		cpos.x = pos.x + (width/2);
		cpos.y = pos.y + (height/2);
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

	Point Rect::getRandomPoint() const{
		std::uniform_real_distribution<double> rx(pos.x, pos.x + width);
		std::uniform_real_distribution<double> ry(pos.y, pos.y + height);

		return Point(rx(sp_rng), ry(sp_rng));
	}

	Portal::Portal(const Point& _p1, const Point& _p2, Polygon *_from, Polygon *_to)
	: p1(_p1), p2(_p2){
		from_id = _from->id;
		to_id = _to->id;
		center.setAsMiddle(p1, p2);
		roughDistance = _from->center().squareDistanceTo(center) + center.squareDistanceTo(_to->center());

		width = p1.distanceTo(p2);
		unit.x = (p2.x - p1.x) / width;
		unit.y = (p2.y - p1.y) / width;
	}

	Portal* Polygon::getNeighbor(int to_id){
		for(Portal& portal : neighbors){
			if (portal.to_id == to_id)
				return &portal;
		}

		return NULL;
	}

	bool Polygon::contains(const Point& tp) const{
		// W. Randolph Franklin's PNPoly test
		int num_nodes = points.size();
		bool c = false;

		for (int i = 0, j = num_nodes-1; i < num_nodes; j = i++) {
			const Point& pi = points[i]; const Point& pj = points[j];

			if ( ((pi.y>tp.y) != (pj.y>tp.y)) && (tp.x < (pj.x-pi.x) * (tp.y-pi.y) / (pj.y-pi.y) + pi.x) )
				c = !c;
	  	}

	  	return c;
	}

	void orderPoint(Point& p1, Point& p2, int i1, int i2, int size, bool CCW){
		// p1 is left, p2 is right
		bool test; int last = size - 1;

		if (CCW){
			test = (i1 - i2 == 1);
			test = test || ((i1 == 0) && (i2 == last)); // edge case
		} else {
			test = (i2 - i1 == 1);
			test = test || ((i2 == 0) && (i1 == last)); // edge case
		}

		if (!test){
			Point temp = p1;
			p1 = p2;
			p2 = temp;
		}
	}

	bool Polygon::testNeighborhood(const Polygon& poly, Point& result_p1, Point& result_p2, bool CCW) const{
		// TODO optimize this
		int match = 0; auto pmatch = points.end(); int i = 0, i1 = 0, i2 = 0;

		for (auto p1 = points.begin(); p1 != points.end(); ++p1){
			for (auto p2 = poly.points.begin(); p2 != poly.points.end(); ++p2){
				if (*p1 == *p2){
					if (match == 0){
						pmatch = p1;
						match = 1;
						i1 = i;
					} else {
						i2 = i;
						result_p1 = *pmatch;
						result_p2 = *p1;
						orderPoint(result_p1, result_p2, i1, i2, points.size(), CCW);
						return true;
					}
				}
			}
			++i;
		}

		return false;
	}

	void Polygon::calcCentroid(){
		double Cx = 0.0, Cy = 0.0, A = 0.0;
		double x, y, x1, y1, a; int n = points.size();

		for (int i = 0; i < n - 1; ++i){
			x = points[i].x; y = points[i].y;
			x1 = points[i+1].x; y1 = points[i+1].y;

			a = (x*y1 - x1*y);
			Cx += (x + x1) * a;
			Cy += (y + y1) * a;
			A += a;
		}

		// last point & first, closed polygon
		x = points[n-1].x; y = points[n-1].y;
		x1 = points[0].x; y1 = points[0].y;

		a = (x*y1 - x1*y);
		Cx += (x + x1) * a;
		Cy += (y + y1) * a;
		A += a;

		A = 0.5*A;
		Cx = Cx / (6.0*A);
		Cy = Cy / (6.0*A);
		centroid.x = Cx; centroid.y = Cy;
		area = A; densityWeight = 0;

		if (area < 0){
			area = -area;
			orientation = ORIENTATION_CW;
		} else if (area > 0){
			orientation = ORIENTATION_CCW;
		} else {
			orientation = 0;
		}

	}

	void Polygon::setOrientation(int _orientation){
		if (orientation == _orientation) return;

		const int n = points.size();
		for (int i = 0; i < n / 2; ++i){
			Point temp = points[i];
			points[i] = points[n-i-1];
			points[n-i-1] = temp;
		}

		orientation = _orientation;
	}

	void Polygon::calcDensityWeight(int numAgent, double radius){
		//double areaUse = numAgent * radius * radius;
		//densityWeight = areaUse / (area - areaUse); // == density / (1 - density)
		double density = (numAgent * 3.14 * radius * radius) / area;
		densityWeight = density / (1 - density);
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

