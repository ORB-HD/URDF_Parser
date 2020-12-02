#include "urdf/geometry.h"
#include "urdf/link.h"

using namespace urdf;

Sphere* Sphere::fromXml(TiXmlElement *xml) {
	Sphere* s = new Sphere;

	if (xml->Attribute("radius") != nullptr){
		try{
			s->radius = boost::lexical_cast<double>(xml->Attribute("radius"));
		} catch (boost::bad_lexical_cast &e) {
			delete s;
			std::ostringstream error_msg;
			error_msg << "Error while parsing link '" << getParentLinkName(xml)
			          << "': sphere radius [" << xml->Attribute("radius")
			          << "] is not a valid float: " << e.what() << "!";
			throw URDFParseError(error_msg.str());
		}
	} else {
		delete s;
		std::ostringstream error_msg;
		error_msg << "Error while parsing link '" << getParentLinkName(xml)
		          << "': Sphere shape must have a radius attribute";
		throw URDFParseError(error_msg.str());
	}

	return s;
}

Box* Box::fromXml(TiXmlElement *xml) {
	Box* b = new Box;

	if (xml->Attribute("size") != nullptr) {
		try{
			b->dim = Vector3::fromVecStr(xml->Attribute("size"));
		}catch (URDFParseError &e) {
			delete b;
			std::ostringstream error_msg;
			error_msg << "Error while parsing link '" << getParentLinkName(xml)
					  << "': box size [" << xml->Attribute("size")
					  << "] is not a valid: " << e.what() << "!";
			throw URDFParseError(error_msg.str());
		}
	} else {
		delete b;
		std::ostringstream error_msg;
		error_msg << "Error while parsing link '" << getParentLinkName(xml)
				  << "': Sphere shape must have a size attribute";
		throw URDFParseError(error_msg.str());
	}

	return b;
}

Cylinder* Cylinder::fromXml(TiXmlElement *xml) {
	Cylinder* y = new Cylinder;

	if (xml->Attribute("length") != nullptr && xml->Attribute("radius") != nullptr) {
		try {
			y->length = boost::lexical_cast<double>(xml->Attribute("length"));
		} catch (boost::bad_lexical_cast &e) {
			delete y;
			std::ostringstream error_msg;
			error_msg << "Error while parsing link '" << getParentLinkName(xml)
					  << "': cylinder length [" << xml->Attribute("length")
					  << "] is not a valid float: " << e.what() << "!";
			throw URDFParseError(error_msg.str());
		}

		try{
			y->radius = boost::lexical_cast<double>(xml->Attribute("radius"));
		} catch (boost::bad_lexical_cast &e) {
			delete y;
			std::ostringstream error_msg;
			error_msg << "Error while parsing link '" << getParentLinkName(xml)
					  << "': cylinder radius [" << xml->Attribute("radius")
					  << "] is not a valid float: " << e.what() << "!";
			throw URDFParseError(error_msg.str());
		}
	} else {
		delete y;
		std::ostringstream error_msg;
		error_msg << "Error while parsing link '" << getParentLinkName(xml)
				  << "': Cylinder shape must have both length and radius attributes!";
		throw URDFParseError(error_msg.str());
	}

	return y;
}


Mesh* Mesh::fromXml(TiXmlElement *xml) {
	Mesh* m = new Mesh;

	if (xml->Attribute("filename") != nullptr) {
		m->filename = xml->Attribute("filename");
	} else {
		delete m;
		std::ostringstream error_msg;
		error_msg << "Error while parsing link '" << getParentLinkName(xml)
				  << "Mesh must contain a filename attribute!";
		throw URDFParseError(error_msg.str());
	}

	if (xml->Attribute("scale") != nullptr) {
		try {
			m->scale = Vector3::fromVecStr(xml->Attribute("scale"));
		} catch (URDFParseError &e) {
			delete m;
			std::ostringstream error_msg;
			error_msg << "Error while parsing link '" << getParentLinkName(xml)
					  << "': mesh scale [" << xml->Attribute("scale")
					  << "] is not a valid: " << e.what() << "!";
			throw URDFParseError(error_msg.str());
		}
	}
	return m;
}

Geometry* Geometry::fromXml(TiXmlElement *xml) {
	if (xml == nullptr) {
		std::ostringstream error_msg;
		error_msg << "Error while parsing link '" << getParentLinkName(xml)
		          << "' geometry structure pointer is null nothing to parse!";
		throw URDFParseError(error_msg.str());
	}

	TiXmlElement *shape = xml->FirstChildElement();
	if (shape == nullptr) {
		std::ostringstream error_msg;
		error_msg << "Error while parsing link '" << getParentLinkName(xml)
		          << "' geometry does not contain any shape information!";
		throw URDFParseError(error_msg.str());
	}

	const std::string type_name = shape->ValueTStr().c_str();
	if (type_name == "sphere") {
		return Sphere::fromXml(shape);
	} else if (type_name == "box") {
		return Box::fromXml(shape);
	} else if (type_name == "cylinder") {
		return Cylinder::fromXml(shape);
	} else if (type_name == "mesh") {
		return Mesh::fromXml(shape);
	} else {
		std::ostringstream error_msg;
		error_msg << "Error while parsing link '" << getParentLinkName(xml)
		          << "' unknown shape type '" << type_name << "'!";
		throw URDFParseError(error_msg.str());
	}
}
