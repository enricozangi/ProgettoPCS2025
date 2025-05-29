#pragma once

#include "polyhedron_library.hpp"
#include <vector>
#include <string>

using namespace polyhedron_library;
using namespace std;

// check input validity

string controllaQuadrupla(const vector<int>& quadrupla);

// initialize platonic solids

Polyhedron tetraedro();
Polyhedron ottaedro();
Polyhedron icosaedro();

// normalize vertices

double norm(const Vertex& v);
Vertex normalize(const Vertex& v);

// export functions

void exportVertices(const vector<Vertex>& vertices);
void exportEdges(const vector<Edge>& edges);
void exportFaces(const vector<Face>& faces);
void exportPolyhedra(const Polyhedron& p);



