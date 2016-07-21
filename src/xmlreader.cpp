#include "xmlreader.h"
#include "helper.h"
#include "tinyxml2/tinyxml2.h"
#include <cstdlib>

using tinyxml2::XML_SUCCESS; 
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

namespace POICS{

	class POICS_API XMLMapReaderImpl : public XMLMapReader {
	private:
		tinyxml2::XMLDocument doc;
	public:
		XMLMapReaderImpl(const char* _filepath);
		virtual ~XMLMapReaderImpl(){}

		virtual void build (MapArea& map);
	};

	class POICS_API XMLAgentReaderImpl : public XMLAgentReader {
	private:
		tinyxml2::XMLDocument doc;
	public:
		XMLAgentReaderImpl(const char* _filepath);
		virtual ~XMLAgentReaderImpl(){}

		virtual void build (AgentBuilder& ab);
	};

	XMLMapReader* XMLMapReader::create(const char* filepath){
		return new XMLMapReaderImpl(filepath);
	}

	XMLAgentReader* XMLAgentReader::create(const char* filepath){
		return new XMLAgentReaderImpl(filepath);
	}

	typedef std::vector<std::string> token_t;

	const char* POLY_SHAPE = "poly";
	const char* RECT_SHAPE = "rect";

	double XMLMapReader::MAP_SCALE = 1.0;

	double scale(double& d){
		return d *= XMLMapReader::MAP_SCALE;
	}

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

	void dataElmt(XMLElement *elmt, std::string& key, std::string& value){
		strAttr(elmt, "key", key);
		strAttr(elmt, "value", value);
	}

	void readRect(XMLElement *elmt, Rect& r){
		double x, y, w, h;

		if ((elmt->QueryDoubleAttribute("x", &x) != XML_SUCCESS) || \
			(elmt->QueryDoubleAttribute("y", &y) != XML_SUCCESS) || \
			(elmt->QueryDoubleAttribute("width", &w) != XML_SUCCESS) || \
			(elmt->QueryDoubleAttribute("height", &h) != XML_SUCCESS))
			except("Expecting attributes x, y, width and height");

		r.set(scale(x),scale(y),scale(w),scale(h));
	}

	void readPoly(XMLElement *elmt, Polygon& poly){
		std::string s; char buf; poly.reset(); Point point; double x, y;
		trim(s.assign(elmt->GetText()));

		token_t tokens;
		split(s, ' ', tokens);

		std::stringstream ss;

		for (std::string& strp : tokens){
			if (strp.length() == 0) continue;

			ss.str(strp);
			ss >> x >> buf >> y;

			point.x = scale(x);
			point.y = scale(y);

			poly.addPoint(point);
			ss.clear();
		}
	}

	XMLMapReaderImpl::XMLMapReaderImpl(const char* _filepath) {
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

	void XMLMapReaderImpl::build (MapArea& map){
		XMLElement *elmt1, *elmt2, *elmt3; std::string s1; Rect rect;

		elmt1 = readChildElement(&doc, "environment");

		double d1, d2;

		try {
			doubleAttr(elmt1, "scale", d1);
			if (d1 > 0)
				MAP_SCALE = d1;
		} catch (...){}

		doubleAttr(elmt1, "width", d1);
		doubleAttr(elmt1, "height", d2);
		map.width = scale(d1);
		map.height = scale(d2);

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
			doubleAttr(elmt3, "dist", d1);
			readRect(elmt3, rect);
			map.addSpawnPoint(d1, rect);
			elmt3 = elmt3->NextSiblingElement("spawn");
		}while(elmt3 != NULL);


		/** read exit points **/
		elmt2 = readChildElement(elmt1, "exits");
		elmt3 = readChildElement(elmt2, "exit");

		do{
			doubleAttr(elmt3, "dist", d1);
			readRect(elmt3, rect);
			map.addExitPoint(d1, rect);
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
				rect.copyToPolygonCCW(poly);
			} else {
				except("Unknown shape type attribute");
			}

			poly.calcCentroid(); poly.setOrientation(ORIENTATION_CCW);

			map.addObstacle(poly);
			elmt3 = elmt3->NextSiblingElement("obstacle");
		}
	}

	XMLAgentReaderImpl::XMLAgentReaderImpl(const char* _filepath) {
		doc.LoadFile(_filepath);
	}

	void readMinMax(XMLElement *elmt, double& min, double& max){
		doubleAttr(elmt, "min", min); doubleAttr(elmt, "max", max);
	}

	void XMLAgentReaderImpl::build (AgentBuilder& as){
		XMLElement *elmt1, *elmt2, *elmt3, *elmt4, *elmt5; std::string s1, s2; int n; double d1, d2;

		elmt1 = readChildElement(&doc, "agents");

		intAttr(elmt1, "count", n); as.setNumAgents(n);
		intAttr(elmt1, "enterTime", n); as.entryTime = n;

		elmt2 = readChildElement(elmt1, "profiles");

		elmt3 = readChildElement(elmt2, "profile");
		do{
			strAttr(elmt3, "name", s1);
			doubleAttr(elmt3, "dist", d1);
			int id = as.addProfile (s1, d1);

			elmt4 = readChildElement(elmt3, "maxTime");
			readMinMax(elmt4, d1, d2);
			as.setProfileDuration(id, d1, d2);

			elmt4 = readChildElement(elmt3, "interest");

			elmt5 = readChildElement(elmt4, "topic");
			do {
				strAttr(elmt5, "name", s1);
				readMinMax(elmt5, d1, d2);
				as.addInterestRange(id, s1, d1, d2);
				elmt5 = elmt5->NextSiblingElement("topic");
			} while (elmt5 != NULL);

			elmt4 = readChildElement(elmt3, "data", true);
			while (elmt4 != NULL){
				dataElmt(elmt4, s1, s2);
				as.addProfileExtras(id, s1, s2);
				elmt4 = elmt4->NextSiblingElement("data");
			}
			
			elmt3 = elmt3->NextSiblingElement("profile");
		}while(elmt3 != NULL);
	}
}