#ifndef URDF_LINK_H
#define URDF_LINK_H

#include <string>
#include <vector>
#include <map>
#include <optional>

#include "urdf/joint.h"
#include "urdf/geometry.h"
#include "urdf/common.h"

namespace urdf{
	struct Material {
		std::string name;
		std::string texture_filename;
		Color color;

		void clear() {
			name.clear();
			texture_filename.clear();
			color.clear();
		}

		Material() { clear(); }
    Material(const Material& m): name(m.name), texture_filename(m.texture_filename),
                                 color(m.color) {}

		static Material fromXml(TiXmlElement* xml, bool);
	};


	struct Inertial {
		Transform origin;
		double mass;
		double ixx,ixy,ixz,iyy,iyz,izz;

		void clear() {
			origin.clear();
			mass = 0.;
			ixx = 0.;
			ixy = 0.;
			ixz = 0.;
			iyy = 0.;
			iyz = 0.;
			izz = 0;
		}

		Inertial() : mass(0.), ixx(0.), ixy(0.), ixz(0.), iyy(0.), iyz(0.), izz(0.) {}
    Inertial(const Inertial& i) : origin(i.origin), ixx(i.ixx), ixy(i.ixy), ixz(i.ixz),
                                  iyy(i.iyy), iyz(i.iyz), izz(i.izz) {}

		static Inertial fromXml(TiXmlElement* xml);
	};

	struct Visual {
		std::string name;
		std::string material_name;
		Transform origin;

		std::optional<std::shared_ptr<Geometry>> geometry;
		std::optional<Material> material;

		void clear() {
			origin.clear();
			name.clear();
			material_name.clear();

			material.reset();
			geometry.reset();
		}

		Visual() { this->clear(); }
    Visual(const Visual& v) : name(v.name), material_name(v.material_name),
                              origin(v.origin), geometry(v.geometry), material(v.material) {}

		static Visual fromXml(TiXmlElement* xml);
	};

	struct Collision {
		std::string name;
		Transform origin;
		std::optional<std::shared_ptr<Geometry>> geometry;

		void clear() {
			name.clear();
			origin.clear();

			geometry.reset();
		}

		Collision() { this->clear(); }
    Collision(const Collision& c) : name(c.name), origin(c.origin), geometry(c.geometry) {}

		static Collision fromXml(TiXmlElement* xml);
	};

	const char* getParentLinkName(TiXmlElement* xml);

	struct Link {
		std::string name;

		std::optional<Inertial> inertial;

		std::vector<Collision>  collisions;
		std::vector<Visual>  visuals;

		Joint* parent_joint;
		Link* parent_link;

		std::vector<Joint*> child_joints;
		std::vector<Link*> child_links;

		int link_index;

		Link* getParent() const {
			return parent_link;
		}

		void setParentLink(Link* parent) {
			parent_link = parent;
		}

		void setParentJoint(Joint* parent) {
			parent_joint = parent;
		}

		void clear() {
			name.clear();
			link_index=-1;

			child_joints.clear();
			child_links.clear();
			collisions.clear();
			visuals.clear();

			inertial.reset();

			parent_joint = nullptr;
			parent_link = nullptr;
		}

		Link() { this->clear(); }
    Link(const Link& l) : name(l.name), inertial(l.inertial), collisions(l.collisions),
                          visuals(l.visuals), parent_joint(l.parent_joint),
                          parent_link(l.parent_link), child_joints(l.child_joints),
                          child_links(l.child_links), link_index(l.link_index) {}

		static Link fromXml(TiXmlElement *xml);
	};

}
#endif
