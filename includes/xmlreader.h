#ifndef MAPREADER_H
#define MAPREADER_HPP

#include "dllmacro.h"
#include "mapobject.h"
#include "agentbuilder.h"

namespace POICS {

	class POICS_API XMLMapReader {
	public:
		static double MAP_SCALE; // default is 1

		XMLMapReader(){}
		virtual ~XMLMapReader(){}

		virtual void build (MapArea& map) = 0;

		static XMLMapReader* create(const char* filepath);
	};

	class POICS_API XMLAgentReader {
	public:
		XMLAgentReader(){}
		virtual ~XMLAgentReader(){}

		virtual void build (AgentBuilder& ab) = 0;

		static XMLAgentReader* create(const char* filepath);
	};
}

#endif