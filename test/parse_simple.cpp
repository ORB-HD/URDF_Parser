#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "urdf/model.h"
#include <string>
#include <iostream>
#include <exception>
#include "tinyxml/txml.h"

const char* urdfstr =
    "<?xml version=\"1.0\">\n"
    "<robot name=\"test\">\n"
    "<link name=\"base_link\">\n"
    "<visual>"
    "<geometry>"
    "<cylinder length=\"0.6\" radius=\"0.2\"/>"
    "</geometry>"
    "</visual>"
    "</link>"
    "</robot>";

TEST_CASE ( "load a urdf model and check for correct structure", "[UrdfModel]" ) {
    try {
        auto model = urdf::UrdfModel::fromUrdfStr(std::string(urdfstr));
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

}
