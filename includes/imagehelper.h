#ifndef IMAGEHELPER_H
#define IMAGEHELPER_H

#include "dllmacro.h"
#include "shapes.h"

namespace POICS{
	class POICS_API Painter{
	public:
		Painter(){}

		virtual ~Painter(){}

		virtual void setColor(unsigned char r, unsigned char g, unsigned char b) = 0;
		virtual void drawPoly(Polygon& poly) = 0;
		virtual void floodFill(Point& start, unsigned char rborder, unsigned char gborder, unsigned char bborder) = 0;
		virtual void drawRect(Rect& rect) = 0;
		virtual void drawLine(Point& p1, Point& p2) = 0;
		virtual void drawPoint(Point& p) = 0;
		virtual void save(const char* filename) = 0;

		static Painter* create(double w, double h, double _scale = 1.0);
	};
}

#endif