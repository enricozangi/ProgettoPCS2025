#pragma once

#include "polyhedron_library.hpp"
#include <vector>
#include <string>

using namespace polyhedron_library;
using namespace std;
namespace polyhedron_library {
    std::string controllaQuadrupla(const std::vector<int>& quadrupla);
}

using polyhedron_library::controllaQuadrupla;

namespace polyhedron_library {
    Polyhedron tetraedro();
    Polyhedron ottaedro();
    Polyhedron icosaedro();
}