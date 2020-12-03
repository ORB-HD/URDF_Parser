#ifndef URDF_MODEL_H
#define URDF_MODEL_H

#include <string>
#include <map>

#include "urdf/common.h"
#include "urdf/exception.h"
#include "urdf/link.h"
#include "urdf/joint.h"

using namespace std;
using namespace boost;

namespace urdf {

	struct UrdfModel {
		string name;
		Link* root_link;

		map<string, Link*> link_map;
		map<string, Joint*> joint_map;
		map<string, Material*> material_map;

		vector<Link> links;
		vector<Joint> joints;
		vector<Material> materials;


		const string& getName() const { return name; }
		Link* getRoot() const { return root_link; }

		Link* getLink(const string& name);
		Joint* getJoint(const string& name);
		Material* getMaterial(const string& name);

		void getLinks(vector<Link*>& linklist) const;

		void clear() {
			name.clear();

			link_map.clear();
			links.clear();

			joints.clear();
			joint_map.clear();

			materials.clear();
			material_map.clear();

			root_link = nullptr;
		};


		void initLinkTree(map<string, string>& parent_link_tree);
		void findRoot(const map<string, string> &parent_link_tree);

		UrdfModel() { clear(); }

		static UrdfModel* fromUrdfStr(const std::string& xml_string);
	};

}

#endif
