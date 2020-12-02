#ifndef URDF_MODEL_H
#define URDF_MODEL_H

#include <string>
#include <map>

#include "urdf/exception.h"
#include "urdf/link.h"

using namespace std;
using namespace boost;

namespace urdf {

	struct UrdfModel {
		string name;
		Link* root_link;

		int m_numLinks;	//includes parent
		int m_numJoints;

		map<string, Link> links;
		map<string, Joint> joints;
		map<string, Material> materials;


		const string& getName() const { return name; }
		const Link* getRoot() const { return root_link; }

		const Link* getLink(const string& name) const;
		const Joint* getJoint(const string& name) const;
		const Material* getMaterial(const string& name) const;

		void getLinks(vector<const Link*>& linklist) const;

		void clear() {
			m_numLinks=0;
			m_numJoints = 0;

			name.clear();
			this->links.clear();
			this->joints.clear();
			this->materials.clear();
			this->root_link = nullptr;
		};


		void initLinkTree(map<string, string>& parent_link_tree);
		void findRoot(const map<string, string> &parent_link_tree);

		UrdfModel() { clear(); }

		static UrdfModel* fromUrdfStr(const std::string& xml_string);
	};

}

#endif
