#ifndef IMAGEHELPER_H
#define IMAGEHELPER_H

#include "polypartition/image.h"
#include "polypartition/imageio.h"
#include "shapes.h"

namespace POICS{
	class Painter{
	private:
		Image image;
		double scale;
		Image::Pixel color;

	public:
		Painter(double width, double height, double _scale = 1.0): scale(_scale){
			image.Init((int) (width*scale), (int) (height*scale));

			Image::Pixel white={255,255,255};
			image.Clear(white);
			setColor(0,0,0);
		}

		~Painter(){}

		void setColor(unsigned char r, unsigned char g, unsigned char b){
			color.R = r; color.G = g; color.B = b;
		};
		

		void drawPoly(Polygon& poly){
			std::vector<Point>& points = poly.getPoints();
			int n = points.size();

			Point *p1, *p2;

			for (int i = 0; i < n; ++i){
				p1 = &(points[i]);

				if (i == n-1){
					p2 = &(points[0]);
				} else {
					p2 = &(points[i+1]);
				}

				drawLine(*p1, *p2);
			}
		}

		void floodFill(Point& start, unsigned char rborder, unsigned char gborder, unsigned char bborder){
			//Image::Pixel Image::GetPixelColor(long x,long y)
			// TODO kalo yg lain udah
		}

		void drawRect(Rect& rect){
			Polygon pl;
			rect.copyToPolygonCW(pl);
			drawPoly(pl);
		}

		void drawLine(Point& p1, Point& p2){
			int x1, y1, x2, y2;

			x1 = (int) (p1.x * scale);
			y1 = (int) (p1.y * scale);
			x2 = (int) (p2.x * scale);
			y2 = (int) (p2.y * scale);

			image.DrawLine(x1,y1,x2,y2,color);
		}

		void save(const char* filename){
			ImageIO io;
			io.SaveImage(filename, &image);
		}
	};
}

#endif