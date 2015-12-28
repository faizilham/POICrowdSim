#include "mapcompiler.h"
#include "polypartition/polypartition.h"
#include "helper.h"
#include <list>
#include <cstdio>
#include <iostream>

using std::list;

namespace POICS {
	void toTPPLPoly(Polygon& poly, bool hole, TPPLPoly& tpl){
		std::vector<Point>& points = poly.getPoints();
		tpl.Init(points.size());

		int i = 0;
		for (Point& p : points){
			tpl[i].x = p.x;
			tpl[i].y = p.y;
			++i;
		}

		if(hole) {
			tpl.SetHole(true);
			tpl.SetOrientation(TPPL_CW);
		} else {
			tpl.SetOrientation(TPPL_CCW);
		}
	}

	void toPOICSPoly(TPPLPoly& tpl, Polygon& poly){
		poly.reset();

		int numpoints = tpl.GetNumPoints();
		for (int i = 0; i < numpoints; ++i){
			poly.addPoint(tpl[i].x, tpl[i].y);
		}
	}

	void createTPPList(MapArea& maparea, std::list<TPPLPoly>& tpls){
		TPPLPoly tpl; tpls.clear();

		/** add map area polygon **/
		Rect area(0, 0, maparea.width, maparea.height);
		Polygon parea; area.copyToPolygonCCW(parea);

		toTPPLPoly(parea, false, tpl);
		tpls.push_back(tpl);

		/** add obstacles **/
		for (Polygon& poly : maparea.getObstacles()){
			toTPPLPoly(poly, true, tpl);
			tpls.push_back(tpl);
		}
	}

	void WritePoly(FILE *fp, TPPLPoly *poly);
	void WritePoly(const char *filename, TPPLPoly *poly);
	void WritePolyList(FILE *fp, list<TPPLPoly> *polys);
	void WritePolyList(const char *filename, list<TPPLPoly> *polys);

	void HMNavMeshGenerator::buildNavMesh(std::vector<Polygon>& result_navmesh){

		TPPLPartition pp;
		std::list<TPPLPoly> input, output;

		/* convert to TPPLPoly */
		createTPPList(maparea, input);

		/* HM Partition */
		if (!pp.ConvexPartition_HM(&input, &output)){
			except("Invalid input polygon when building navmesh");
		}

		WritePolyList("tmp/output.txt", &output);

		/* convert back to POICS::Polygon */
		int i = 0; Polygon poly;
		for (TPPLPoly& tpl : output){
			toPOICSPoly(tpl, poly);
			poly.id = i; result_navmesh.push_back(poly); ++i;
		}

		/* setup neighbor list */
		Point p1, p2;
		int n = result_navmesh.size();
		for (int i = 0; i < n - 1; ++i){
			for (int j = i + 1; j < n; ++j){
				Polygon& poly1 = result_navmesh[i];
				Polygon& poly2 = result_navmesh[j];

				if (poly1.testNeighborhood(poly2, p1, p2)){
					poly1.addNeighbor(poly2, p1, p2);
					poly2.addNeighbor(poly1, p1, p2);
				}
			}
		}

		/** test print **/
		std::cout<<n<<std::endl;
		for (Polygon& pl: result_navmesh){
			std::cout<<pl.id<<":";
			for (Portal& portal : pl.getNeighbors()){
				std::cout<<portal.neighbor->id<<" ";
			}
			std::cout<<std::endl;
		}
	}

	void WritePoly(FILE *fp, TPPLPoly *poly) {
		int i,numpoints;
		numpoints = poly->GetNumPoints();

		fprintf(fp,"%d\n",numpoints);
		
		if(poly->IsHole()) {
			fprintf(fp,"1\n");
		} else {
			fprintf(fp,"0\n");
		}

		for(i=0;i<numpoints;i++) {
			fprintf(fp,"%g %g\n",(*poly)[i].x, (*poly)[i].y);
		}
	}

	void WritePoly(const char *filename, TPPLPoly *poly) {
		FILE *fp = fopen(filename,"w");
		if(!fp) {
			printf("Error writing file %s\n", filename);
			return;
		}
		WritePoly(fp,poly);
		fclose(fp);	
	}

	void WritePolyList(FILE *fp, list<TPPLPoly> *polys) {
		list<TPPLPoly>::iterator iter;

		fprintf(fp,"%ld\n",polys->size());

		for(iter = polys->begin(); iter != polys->end(); iter++) {
			WritePoly(fp,&(*iter));
		}
	}

	void WritePolyList(const char *filename, list<TPPLPoly> *polys) {
		FILE *fp = fopen(filename,"w");
		if(!fp) {
			printf("Error writing file %s\n", filename);
			return;
		}
		WritePolyList(fp,polys);
		fclose(fp);	
	}
}
