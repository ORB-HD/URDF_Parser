#include "urdf/model.h"
#include "urdf/link.h"
#include "urdf/joint.h"

using namespace urdf;

const Link* UrdfModel::getLink(const string& name) const {
	if (links.find(name) == links.end()) {
		return nullptr;
	} else {
		return &links.find(name)->second;
	}
}

const Joint* UrdfModel::getJoint(const string& name) const {
	if (joints.find(name) == joints.end()) {
		return nullptr;
	} else {
		return &joints.find(name)->second;
	}
}

const Material* UrdfModel::getMaterial(const string& name) const {
	if (materials.find(name) == materials.end()) {
		return nullptr;
	} else {
		return &materials.find(name)->second;
	}
}

void UrdfModel::getLinks(vector<const Link*>& linklist) const {
	for (auto link = links.begin(); link != links.end(); link++) {
		linklist.push_back(&link->second);
	}
}

void UrdfModel::initLinkTree(map<string, string>& parent_link_tree) {
	for (auto joint = joints.begin(); joint != joints.end(); joint++) {
		string parent_link_name = joint->second.parent_link_name;
		string child_link_name = joint->second.child_link_name;

		if (parent_link_name.empty()){
			ostringstream error_msg;
			error_msg << "Error while constructing model! Joint [" << joint->second.name
					  << "] is missing a parent link specification.";
			throw URDFParseError(error_msg.str());
		}
		if (child_link_name.empty()) {
			ostringstream error_msg;
			error_msg << "Error while constructing model! Joint [" << joint->second.name
					  << "] is missing a child link specification.";
			throw URDFParseError(error_msg.str());
		}

		const Link* child_link = getLink(child_link_name);
		if (child_link == nullptr) {
			ostringstream error_msg;
			error_msg << "Error while constructing model! Child link [" << child_link_name
					  << "] of joint [" <<  joint->first << "] not found";
			throw URDFParseError(error_msg.str());
		}
		const Link* parent_link = getLink(parent_link_name);
		if (parent_link == nullptr) {
			ostringstream error_msg;
			error_msg << "Error while constructing model! Parent link [" << parent_link_name
					  << "] of joint [" <<  joint->first << "] not found";
			throw URDFParseError(error_msg.str());
		}

		child_link->setParentLink(parent_link);
		child_link->setParentJoint(&joint->second);

		parent_link->child_joints.push_back(joint->second);
		parent_link->child_links.push_back(child_link);

		parent_link_tree[child_link->name] = parent_link_name;

	}
}

void UrdfModel::findRoot(const map<string, string> &parent_link_tree) {
	for (auto l=links.begin(); l!=links.end(); l++) {
		auto parent = parent_link_tree.find(l->first);
		if (parent == parent_link_tree.end()) {
			if (!this->root_link) {
				root_link = getLink(l->first);
			} else {
				throw URDFParseError("Two root links found: [" + this->root_link_->name + "] and [" + l->first + "]");
			}
		}
	}
	if (!this->root_link) {
		throw URDFParseError("No root link found. The robot xml is not a valid tree.");
	}
}

UrdfModel* fromUrdfStr(const std::string& xml_string) {
	UrdfModel* model = new URDFModel;

	TiXmlDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error()) {
		delete model;
		std::string error_msg = xml_doc.ErrorDesc();
		xml_doc.ClearError();
		throw URDFParseError(error_msg);
	}

	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (robot_xml == nullptr) {
		std::string error_msg = "Error! Could not find the <robot> element in the xml file";
		throw URDFParseError(error_msg);
	}

	const char *name = robot_xml->Attribute("name");
	if (name != nullptr){
		model->name = std::string(name);
	} else {
		delete model;
		std::string error_msg = "No name given for the robot. Please add a name attribute to the robot element!";
		throw URDFParseError(error_msg);
	}

	for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml != nullptr; material_xml = material_xml->NextSiblingElement("material")) {
		Material material = Material::fromXml(material_xml, false); // material needs to be fully defined here
		if (model->getMaterial(material->name) != nullptr) {
			std::ostringstream error_msg;
			error_msg << "Duplicate materials '" << material->name << "' found!";
			throw URDFParseError(error_msg.str());
		} else {
			model->materials.insert(make_pair(material->name,material));
		}
	}

	for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml != nullptr; link_xml = link_xml->NextSiblingElement("link")) {
		Link link = Link::fromXml(link_xml);
		model->m_numLinks++;
		if (model->getLink(link.name) != nullptr) {
			std::ostringstream error_msg;
			error_msg << "Error! Duplicate links '" << link.name << "' found!";
			throw URDFParseError(error_msg.str());
		} else {
			// set link visual material
			if (!link.visuals.empty()) {
				auto visual = link.visuals[0];
				if (!visual.material_name.empty()) {
					//find the correct material in model
					if (model->getMaterial(visual.material_name)) {
						visual.material = model->getMaterial( visual.material_name.c_str() );
					} else {
						// if no model matrial found use the one defined in the visual
						if (visual.material) {
							model->materials.insert(make_pair(visual.material_name, visual.material));
						} else {
							// no matrial information available for this visual -> error
							std::ostringstream error_msg;
							error_msg << "Error! Link '" << link.name
									  << "' material '" << visual.material_name
									  <<" ' undefined!";
							throw URDFParseError(error_msg.str());
						}
					}
				}
			}
			model->links.insert(make_pair(link.name,link));
		}
	}

	if (model->links.empty()){
		std::string error_msg = "Error! No link elements found in the urdf file.";
		throw URDFParseError(error_msg);
	}

	for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml != nullptr; joint_xml = joint_xml->NextSiblingElement("joint")) {
		Joint joint = Joint::fromXml(joint_xml);
			model->m_numJoints++;

		if (model->getJoint(joint.name) != nullptr) {
			std::ostringstream error_msg;
			error_msg << "Error! Duplicate joints '" << joint.name << "' found!";
			throw URDFParseError(error_msg.str());
		} else {
			model->joints.insert(make_pair(joint.name, joint));
		}
	}

	std::map<std::string, std::string> parent_link_tree;

	model->initLinkTree(parent_link_tree);
	model->findRoot(parent_link_tree);

	return model;
}
