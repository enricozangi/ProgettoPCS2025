#pragma once

#include "polyhedron_library.hpp"
#include <vector>
#include <string>

using namespace polyhedron_library;
using namespace std;

string controllaQuadrupla(const vector<int>& quadrupla);

Polyhedron tetraedro();
Polyhedron ottaedro();
Polyhedron icosaedro();

double norm (const Vertex& v);
Vertex normalize(const Vertex& v);

