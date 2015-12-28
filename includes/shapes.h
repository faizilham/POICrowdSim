#ifndef SHAPES_H
#define SHAPES_H

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
	};

	class Rect {
	public:
		Point pos; double width, height;
		Rect(double x, double y, double w, double h): pos(x,y), width(w), height(h) {}
	};

	class Polygon;
	class Portal{
	public:
		Point p1; Point p2; Polygon* neighbor;
		Portal(const Point& _p1, const Point& _p2, Polygon* _neighbor): p1(_p1), p2(_p2), neighbor(_neighbor){}
	};

	class Polygon {
	private:
		std::vector<Point> points;
		std::vector<Portal> neighbors;

	public: 
		int id;
		Polygon(int _id) : id(_id){}
		~Polygon(){}

		std::vector<Portal>& getNeighbors() { return neighbors;}
		std::vector<Point>& getPoints() { return points;}

		void addPoint(Point point){ points.push_back(point);}
		void addPoint(double x, double y){ points.push_back(Point(x,y));}

		void addNeighbor(const Point& p1, const Point& p2, Polygon& poly){
			Portal portal(p1, p2, &poly);
			neighbors.push_back(portal);
		}

		bool testNeighborhood(const Polygon& poly, Point& result_p1, Point& result_p2) const;
	};
}

#endif
