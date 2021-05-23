#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "urdf/model.h"
#include <string>
#include <iostream>
#include <exception>
#include "tinyxml/txml.h"

const char* urdfstr_geometry_test =
    "<?xml version=\"1.0\">\n"
    "<robot name=\"test\">\n"
    "<link name=\"base_link\">\n"
    "<visual>"
    "<geometry>"
    "<cylinder length=\"0.6\" radius=\"0.25\"/>"
    "</geometry>"
    "</visual>"
    "<visual>"
    "<geometry>"
    "<sphere radius=\"0.5\"/>"
    "</geometry>"
    "</visual>"
    "<visual>"
    "<geometry>"
    "<box size=\"0.1 0.2 0.3\"/>"
    "</geometry>"
    "</visual>"
    "<visual>"
    "<geometry>"
    "<mesh scale=\"0.7 0.8 0.9\" filename=\"test.obj\" />"
    "</geometry>"
    "</visual>"
    "</link>"
    "</robot>";

using namespace urdf;

TEST_CASE ( "load a urdf an check for correct geometry definitions", "[UrdfModel]" ) {
    std::shared_ptr<UrdfModel> model;

    REQUIRE_NOTHROW(model = UrdfModel::fromUrdfStr(std::string(urdfstr_geometry_test)));

    //base link must exist
    auto base_link = model->getLink("base_link");
    REQUIRE( base_link != nullptr );
    CHECK(base_link->name == "base_link");

    //test visuals
    auto &visuals = base_link->visuals;
    CHECK( visuals.size() == 4 );
    for (auto v : visuals) {
        switch ((*v->geometry.value()).type) {
            case GeometryType::CYLINDER : {
                auto cylinder = (std::shared_ptr<Cylinder>&) v->geometry;

                CHECK( cylinder->length == 0.6 );
                CHECK( cylinder->radius == 0.25 );
                break;
            };
            case GeometryType::SPHERE : {
                auto sphere = (std::shared_ptr<Sphere>&) v->geometry;

                CHECK( sphere->radius == 0.5 );
                break;
            };
            case GeometryType::BOX : {
                auto box = (std::shared_ptr<Box>&) v->geometry;

                CHECK(box->dim.x == 0.1);
                CHECK(box->dim.y == 0.2);
                CHECK(box->dim.z == 0.3);
                break;
            };
            case GeometryType::MESH : {
                auto mesh = (std::shared_ptr<Mesh>&) v->geometry;

                CHECK(mesh->scale.x == 0.7);
                CHECK(mesh->scale.y == 0.8);
                CHECK(mesh->scale.z == 0.9);
                CHECK(mesh->filename == "test.obj");
                break;
            };
        }
    }
}

const char* urdfstr_two_segment =
    "<?xml version=\"1.0\">\n"
    "<robot name=\"test\">\n"
    "  <material name=\"Grey\">\n"
    "    <color rgba=\"0.2 0.2 0.2 1.0\"/>"
    "  </material>"
    "  <link name=\"link_0\">\n"
    "    <inertial>\n"
    "      <origin rpy=\"0 0 0\" xyz=\"-0.1 0 0.07\"/>\n"
    "      <mass value=\"5.0\"/>\n"
    "      <inertia ixx=\"0.05\" ixy=\"0\" ixz=\"0\" iyy=\"0.06\" iyz=\"0\" izz=\"0.03\"/>\n"
    "    </inertial>\n"
    "    <visual>\n"
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
    "      <geometry>\n"
    "        <mesh filename=\"meshes/link_0.obj\"/>\n"
    "      </geometry>\n"
    "      <material name=\"Grey\"/>\n"
    "    </visual>\n"
    "  </link>\n"
    "<!-- joint between link_0 and link_1 -->\n"
    "  <joint name=\"joint_1\" type=\"revolute\">\n"
    "    <parent link=\"link_0\"/>\n"
    "    <child link=\"link_1\"/>\n"
    "    <origin rpy=\"0 0 0\" xyz=\"0 0 0.1575\"/>\n"
    "    <axis xyz=\"0 0 1\"/>\n"
    "    <limit effort=\"300\" lower=\"-2.96705972839\" upper=\"2.96705972839\" velocity=\"10\"/>\n"
    "    <dynamics damping=\"0.5\"/>\n"
    "  </joint>\n"
    "  <link name=\"link_1\">\n"
    "    <inertial>\n"
    "      <origin rpy=\"0 0 0\" xyz=\"0 -0.03 0.12\"/>\n"
    "      <mass value=\"4\"/>\n"
    "      <inertia ixx=\"0.1\" ixy=\"0\" ixz=\"0\" iyy=\"0.09\" iyz=\"0\" izz=\"0.02\"/>\n"
    "    </inertial>\n"
    "    <visual>\n"
    "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
    "      <geometry>\n"
    "        <mesh filename=\"meshes/link_1.obj\"/>\n"
    "      </geometry>\n"
    "      <material name=\"Blue\"/>\n"
    "    </visual>\n"
    "  </link>\n"
    "</robot>";

TEST_CASE ( "load a two segment model with inertial information", "[UrdfModel]" ) {
    std::shared_ptr<UrdfModel> model;

    REQUIRE_NOTHROW(model = UrdfModel::fromUrdfStr(std::string(urdfstr_two_segment)));

    // check material definitons
    CHECK(model->material_map.size() == 2 );
    auto m_grey = model->material_map["Grey"];
    CHECK(m_grey != nullptr);
    CHECK(m_grey->color.r == 0.2f);
    CHECK(m_grey->color.g == 0.2f);
    CHECK(m_grey->color.b == 0.2f);
    CHECK(m_grey->color.a == 1.0f);

    // all link found as expected
    REQUIRE( model->getRoot() != nullptr );
    CHECK(model->getRoot()->name == "link_0");

    auto link1 = model->getLink("link_1");
    CHECK(link1 != nullptr);
    //check inertial info of link
    CHECK(link1->inertial != nullopt);
    auto inertial = &link1->inertial.value();
    CHECK(inertial != nullptr);
    CHECK(inertial->mass == 4.);
    CHECK(inertial->ixx == 0.1);
    CHECK(inertial->ixy == 0.);
    CHECK(inertial->ixz == 0.);
    CHECK(inertial->iyy == 0.09);
    CHECK(inertial->iyz == 0.);
    CHECK(inertial->izz == 0.02);
    CHECK(inertial->origin.rotation.x == 0.);
    CHECK(inertial->origin.rotation.y == 0.);
    CHECK(inertial->origin.rotation.z == 0.);
    CHECK(inertial->origin.rotation.w == 1.);
    CHECK(inertial->origin.position.x == 0.);
    CHECK(inertial->origin.position.y == -0.03);
    CHECK(inertial->origin.position.z == 0.12);

    // check model linkage
    CHECK(model->joint_map.size() == 1);
    auto joint = model->getJoint("joint_1");
    CHECK(joint != nullptr);
    CHECK(joint->parent_link_name == "link_0");
    CHECK(joint->child_link_name == "link_1");
    CHECK(joint->type == JointType::REVOLUTE);
    CHECK(joint->axis.x == 0.);
    CHECK(joint->axis.y == 0.);
    CHECK(joint->axis.z == 1.);
    CHECK(joint->parent_to_joint_transform.rotation.x == 0.);
    CHECK(joint->parent_to_joint_transform.rotation.y == 0.);
    CHECK(joint->parent_to_joint_transform.rotation.z == 0.);
    CHECK(joint->parent_to_joint_transform.rotation.w == 1.);
    CHECK(joint->parent_to_joint_transform.position.x == 0.);
    CHECK(joint->parent_to_joint_transform.position.y == 0.);
    CHECK(joint->parent_to_joint_transform.position.z == 0.1575);

    CHECK(joint->safety == nullopt);
    CHECK(joint->calibration == nullopt);
    CHECK(joint->mimic == nullopt);

    CHECK(joint->limits != nullopt);
    CHECK(joint->limits->get()->lower == -2.96705972839);
    CHECK(joint->limits->get()->upper == 2.96705972839);
    CHECK(joint->limits->get()->effort == 300);
    CHECK(joint->limits->get()->velocity == 10);

    CHECK(joint->dynamics != nullopt);
    CHECK(joint->dynamics->get()->damping == 0.5);
    CHECK(joint->dynamics->get()->friction == 0.);

};
