#include "mapreader.h"
#include <exception>
#include "stringhelper.h"

typedef std::vector<std::string> token_t;

POICS::XMLMapReader::XMLMapReader(std::string _filepath) {
	doc.LoadFile(_filepath);
}

void except(const char* str){
	throw std::runtime_exception(str);
}


POICS::XMLMapReader& POICS::XMLMapReader::operator>> (MapArea& map){
	XMLElement *elmt1, *elmt2; std::string s1, s2;
	if (!(elmt1 = doc.FirstChildElement( "environment" )))
		except("Expecting environment tag");

	double d1, d2;
	if (!elmt1->QueryDoubleAttribute("width", &d1) || elmt1->QueryDoubleAttribute("height", &d2))
		except("Expecting width & height attribute in environment");

	map.width = d1; map.height = d2;

	/** read topic **/
	if (!(elmt2 = elmt1->FirstChildElement("topics")))
		except("Expecting topics tag")

	trim(s1.assign(elmt2->GetText()));
	token_t tokens;
	split(s1, ',', tokens);

	for (std::string topic : tokens){
		map.addTopic(topic);
	}

	/** read obstacle **/


	const char* title = titleElement->GetText();
	printf( "Name of play (1): %s\n", title );

	XMLText* textNode = titleElement->FirstChild()->ToText();
	title = textNode->Value();
	printf( "Name of play (2): %s\n", title );

	return doc.ErrorID();
}

/*

XMLElement* titleElement = doc.FirstChildElement( "PLAY" )->FirstChildElement( "TITLE" );
	const char* title = titleElement->GetText();
	printf( "Name of play (1): %s\n", title );

	XMLText* textNode = titleElement->FirstChild()->ToText();
	title = textNode->Value();
	printf( "Name of play (2): %s\n", title );

	return doc.ErrorID();*/