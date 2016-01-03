#include "imagehelper.h"
#include "polypartition/image.h"
#include "polypartition/imageio.h"

namespace POICS{
	class POICS_API PainterImpl : public Painter{
	private:
		Image image;
		int width, height;
		double scale;
		Image::Pixel color;
	public:
		PainterImpl(double w, double h, double _scale = 1.0): scale(_scale){
			width = (int) (w*scale);
			height = (int) (h*scale);

			image.Init(width, height);

			Image::Pixel white={255,255,255};
			image.Clear(white);
			setColor(0,0,0);
		}

		virtual ~PainterImpl(){}

		virtual void setColor(unsigned char r, unsigned char g, unsigned char b){
			color.R = r; color.G = g; color.B = b;
		};
		

		virtual void drawPoly(Polygon& poly){
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

		virtual void floodFill(Point& start, unsigned char rborder, unsigned char gborder, unsigned char bborder){
			//Image::Pixel Image::GetPixelColor(long x,long y)
			// TODO kalo yg lain udah
		}

		virtual void drawRect(Rect& rect){
			Polygon pl;
			rect.copyToPolygonCW(pl);
			drawPoly(pl);
		}

		virtual void drawLine(Point& p1, Point& p2){
			int x1, y1, x2, y2;

			x1 = (int) (p1.x * scale);
			y1 = height - (int) (p1.y * scale);
			x2 = (int) (p2.x * scale);
			y2 = height - (int) (p2.y * scale);

			image.DrawLine(x1,y1,x2,y2,color);
		}

		virtual void drawPoint(Point& p){
			/*int x = (int) (p.x * scale);
			int y = height - (int) (p.y * scale);

			image.SetPixelColor(x, y, color);*/

			Rect r (p.x-0.5, p.y-0.5, 1, 1);
			drawRect(r);
		}

		virtual void save(const char* filename){
			ImageIO io;
			io.SaveImage(filename, &image);
		}
	};

	Painter* Painter::create(double w, double h, double _scale){
		return new PainterImpl(w,h,_scale);
	}
}