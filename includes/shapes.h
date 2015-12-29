#ifndef SHAPES_H
#define SHAPES_H

#include <iostream>
#include <vector>

namespace POICS {
	static const double EPSILON = 1e-6;

	class Point {
	public:
		double x, y;
		Point(): x(0), y(0){}
		Point(double _x, double _y): x(_x), y(_y){}

		double squareDistanceTo(const Point& p2) const;
		double distanceTo(const Point& p2) const;
		bool operator==(const Point& p2) const;

		void setAsMiddle(const Point& p1, const Point& p2);
		friend std::ostream& operator<<(std::ostream& os, const Point& dt);
	};

	class Polygon;
	class Rect {
		Point pos, cpos; double width, height;
		void calcCenterPoint();
	public:
		
		Rect(){}
		Rect(double x, double y, double w, double h): pos(x,y), width(w), height(h) {
			calcCenterPoint();
		}

		friend std::ostream& operator<<(std::ostream& os, const Rect& dt);

		void set(double x, double y, double w, double h);
		void setPos(double x, double y);

		double x() const {return pos.x;}
		double y() const {return pos.y;}
		double w() const {return width;}
		double h() const {return height;}
		Point center() const {return cpos;}

		void copyToPolygonCW(Polygon& pl) const;
		void copyToPolygonCCW(Polygon& pl) const;
	};

	class Portal{
	public:
		Point p1; Point p2; Polygon* neighbor;
		Portal(){}
		Portal(const Point& _p1, const Point& _p2, Polygon* _neighbor): p1(_p1), p2(_p2), neighbor(_neighbor){}
	};

	class Polygon {
	private:
		std::vector<Point> points;
		std::vector<Portal> neighbors;

	public: 
		int id;
		Polygon(){}
		Polygon(int _id) : id(_id){}
		~Polygon(){}

		std::vector<Portal>& getNeighbors() { return neighbors;}
		std::vector<Point>& getPoints() { return points;}

		void reset(){points.clear(); neighbors.clear();}

		void addPoint(Point point){ points.push_back(point);}
		void addPoint(double x, double y){ points.push_back(Point(x,y));}

		void addNeighbor(Polygon& poly, const Point& p1, const Point& p2){
			Portal portal(p1, p2, &poly);
			neighbors.push_back(portal);
		}

		bool contains(const Point& p) const;
		bool testNeighborhood(const Polygon& poly, Point& result_p1, Point& result_p2) const;

		friend std::ostream& operator<<(std::ostream& os, const Polygon& pl);
	};
}

#endif
