#ifndef MAPREADER_H
#define MAPREADER_HPP

#include "mapobject.h"
#include "tinyxml2/tinyxml2.h"

namespace POICS {

	class XMLMapReader {
	private:
		tinyxml2::XMLDocument doc;
	public:
		XMLMapReader(const char* _filepath);
		~XMLMapReader(){}

		XMLMapReader& operator>> (MapArea& map);
	};
}

#endif