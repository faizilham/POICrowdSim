#ifndef MAPREADER_H
#define MAPREADER_HPP

#include "mapobject.h"
#include "tinyxml2/tinyxml2.h"

namespace POICS {

	class XMLMapReader {
	private:
		XMLDocument doc;
	public:
		XMLMapReader(std::string _filepath);
		~XMLMapReader(){}

		XMLMapReader& operator>> (MapArea& map);
	};
}

#endif