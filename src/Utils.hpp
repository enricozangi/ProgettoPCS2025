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
void normalize(Vertex& v);

// export functions

void exportVertices(const vector<Vertex>& vertices);
void exportEdges(const vector<Edge>& edges);
void exportFaces(const vector<Face>& faces);
void exportPolyhedra(const Polyhedron& p);

// export in Paraview

void exportParaview(const Polyhedron& p);

// dual polyhedra

Polyhedron dualPolyhedron(const Polyhedron& p);
void verificaVertici(const Polyhedron& poly, int id1, int id2);
void shortestPath(Polyhedron& poly, int startId, int endId);