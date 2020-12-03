#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "urdf/model.h"
#include <string>
#include <iostream>
#include <exception>
#include <tinyxml/tinyxml.h>

const char* urdfstr =
    "<?xml version=\"1.0\">\n"
    "<robot>\n"
    "\t<link name=\"base_link\">\n"
    "\t</link>\n"
    "</robot>";

TEST_CASE ( "load a urdf model and check for correct structure", "[UrdfModel]" ) {
    try {
        urdf::UrdfModel* model = urdf::UrdfModel::fromUrdfStr(urdfstr);
        delete model;
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

}
