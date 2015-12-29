#include "mapreader.h"
#include "helper.h"
#include <cstdlib>

using tinyxml2::XML_SUCCESS; 
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

namespace POICS{

	typedef std::vector<std::string> token_t;

	const char* POLY_SHAPE = "poly";
	const char* RECT_SHAPE = "rect";

	void doubleAttr(XMLElement *elmt, const char* attr, double& d){
		if (elmt->QueryDoubleAttribute(attr, &d) != XML_SUCCESS)
			except(std::string("Expecting double attribute: ") + attr);
	}

	void intAttr(XMLElement *elmt, const char* attr, int& d){
		if (elmt->QueryIntAttribute(attr, &d) != XML_SUCCESS)
			except(std::string("Expecting int attribute: ") + attr);
	}

	void strAttr(XMLElement *elmt, const char* attr, std::string& str){
		const char* cstr = elmt->Attribute(attr);
		if (cstr == NULL)
			except(std::string("Expecting string attribute: ") + attr);

		str.assign(cstr);
	}

	void readRect(XMLElement *elmt, Rect& r){
		double x, y, w, h;

		if ((elmt->QueryDoubleAttribute("x", &x) != XML_SUCCESS) || \
			(elmt->QueryDoubleAttribute("y", &y) != XML_SUCCESS) || \
			(elmt->QueryDoubleAttribute("width", &w) != XML_SUCCESS) || \
			(elmt->QueryDoubleAttribute("height", &h) != XML_SUCCESS))
			except("Expecting attributes x, y, width and height");

		r.set(x,y,w,h);
	}

	void readPoly(XMLElement *elmt, Polygon& poly){
		std::string s; char buf; poly.reset(); Point point;
		trim(s.assign(elmt->GetText()));

		token_t tokens;
		split(s, ' ', tokens);

		std::stringstream ss;

		for (std::string& strp : tokens){
			ss.str(strp);
			ss>>point.x>>buf>>point.y;
			poly.addPoint(point);
			ss.clear();
		}
	}

	XMLMapReader::XMLMapReader(const char* _filepath) {
		doc.LoadFile(_filepath);
	}

	XMLElement* readChildElement(XMLNode* node, const char* attr, bool optional = false){
		XMLElement *elmt = node->FirstChildElement(attr);
		if (elmt != NULL || optional){
			return elmt;
		} 

		except(std::string("Expecting child: ") + attr);
		return NULL;
	}

	void readRelevance(XMLElement* root, MapArea& map, int id){
		XMLElement *elmt = readChildElement(root, "relevance");
		elmt = readChildElement(elmt, "topic", true);
		std::string name; double value;

		while (elmt != NULL){
			strAttr(elmt, "name", name);
			doubleAttr(elmt, "value", value);
			map.setTopicRelevance(id, name, value);	
			elmt = elmt->NextSiblingElement("topic");
		}		
	}


	XMLMapReader& XMLMapReader::operator>> (MapArea& map){
		XMLElement *elmt1, *elmt2, *elmt3; std::string s1; Rect rect;

		if ((elmt1 = doc.FirstChildElement( "environment" )) == NULL)
			except("Expecting environment tag");

		double d1, d2;
		doubleAttr(elmt1, "width", d1);
		doubleAttr(elmt1, "height", d2);
		map.width = d1;
		map.height = d2;

		/** read topic **/
		elmt2 = readChildElement(elmt1, "topics");

		trim(s1.assign(elmt2->GetText()));
		token_t tokens;
		split(s1, ',', tokens);

		for (std::string& topic : tokens){
			map.addTopic(topic);
		}

		/** read spawn points **/
		elmt2 = readChildElement(elmt1, "spawns");
		elmt3 = readChildElement(elmt2, "spawn");

		do{
			readRect(elmt3, rect);
			map.addSpawnPoint(0, rect);
			elmt3 = elmt3->NextSiblingElement("spawn");
		}while(elmt3 != NULL);


		/** read exit points **/
		elmt2 = readChildElement(elmt1, "exits");
		elmt3 = readChildElement(elmt2, "exit");

		do{
			readRect(elmt3, rect);
			map.addExitPoint(rect);
			elmt3 = elmt3->NextSiblingElement("exit");
		}while(elmt3 != NULL);

		/** read poi **/
		elmt2 = readChildElement(elmt1, "pois");
		elmt3 = readChildElement(elmt2, "poi");

		do{
			int duration, activity_type = 0;
			strAttr(elmt3, "name", s1);
			intAttr(elmt3, "duration", duration);
			
			readRect(elmt3, rect);
			int id = map.addPOI(s1, activity_type, duration, rect);

			readRelevance(elmt3, map, id);

			elmt3 = elmt3->NextSiblingElement("poi");
		}while(elmt3 != NULL);

		elmt2 = readChildElement(elmt1, "obstacles");
		elmt3 = readChildElement(elmt2, "obstacle", true);

		Polygon poly;
		while (elmt3 != NULL){
			strAttr(elmt3, "shape", s1);

			if (s1 == POLY_SHAPE){
				readPoly(elmt3, poly);
			} else if (s1 == RECT_SHAPE){
				readRect(elmt3, rect);
				rect.copyToPolygonCW(poly);
			} else {
				except("Unknown shape type attribute");
			}

			map.addObstacle(poly);
			elmt3 = elmt3->NextSiblingElement("obstacle");
		}

		return *this;
	}
}