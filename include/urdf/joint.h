#ifndef URDF_JOINT_H
#define URDF_JOINT_H

#include <string>
#include <vector>
#include <optional>
#include <tinyxml/tinyxml.h>

#include "urdf/common.h"

namespace urdf{
	struct JointDynamics {
		double damping;
		double friction;

		void clear() {
			damping = 0;
			friction = 0;
		}

		JointDynamics() : damping(0.), friction(0.) {}

		static JointDynamics fromXml(TiXmlElement* xml);
	};

	struct JointLimits {
		double lower;
		double upper;
		double effort;
		double velocity;

		void clear() {
			lower = 0;
			upper = 0;
			effort = 0;
			velocity = 0;
		}

		JointLimits() : lower(0.), upper(0.), effort(0.), velocity(0.) {}

		static JointLimits fromXml(TiXmlElement* xml);
	};

	struct JointSafety {
		double upper_limit;
		double lower_limit;
		double k_position;
		double k_velocity;

		void clear() {
			upper_limit = 0;
			lower_limit = 0;
			k_position = 0;
			k_velocity = 0;
		}

		JointSafety() : upper_limit(0.), lower_limit(0.), k_position(0.), k_velocity(0.) {};

		static JointSafety fromXml(TiXmlElement* xml);
	};

	struct JointCalibration {
		std::optional<double> rising;
		std::optional<double> falling;

		void clear() {
			rising.reset();
			falling.reset();
		}

		JointCalibration() {}
		static JointCalibration fromXml(TiXmlElement* xml);
	};

	struct JointMimic {
		std::string joint_name;
		double offset;
		double multiplier;

		void clear() {
			joint_name = "";
			offset = 0.;
			multiplier = 0.;
		}

		JointMimic() : joint_name(""), offset(0.), multiplier(0.) {}
		static JointMimic fromXml(TiXmlElement* xml);
	};

	enum JointType {
		UNKNOWN,
		REVOLUTE,			//rotation axis
		CONTINUOUS,
		PRISMATIC,		 //translation axis
		FLOATING,
		PLANAR,				//plane normal axis
		FIXED
	};

	struct Joint {
		std::string name;
		JointType type;
		Vector3 axis;
		std::string child_link_name;
		std::string parent_link_name;
		Transform parent_to_joint_transform;

		std::optional<JointDynamics> dynamics;
		std::optional<JointLimits> limits;
		std::optional<JointSafety> safety;
		std::optional<JointCalibration> calibration;
		std::optional<JointMimic> mimic;

		void clear() {
			this->axis.clear();
			this->child_link_name.clear();
			this->parent_link_name.clear();
			this->parent_to_joint_transform.clear();

			this->dynamics.reset();
			this->limits.reset();
			this->safety.reset();
			this->calibration.reset();
			this->type = JointType::UNKNOWN;
		}

		Joint() : type(JointType::UNKNOWN) {}

		static Joint fromXml(TiXmlElement* xml);
  };
}

#endif
