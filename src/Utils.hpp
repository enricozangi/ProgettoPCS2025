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

void exportVertices(const vector<Vertex>& vertices, const string& subfolder);
void exportEdges(const vector<Edge>& edges, const string& subfolder);
void exportFaces(const vector<Face>& faces, const string& subfolder);
void exportPolyhedra(const Polyhedron& p, const string& subfolder);

// export in Paraview

void exportParaview(const Polyhedron& p, const string& subfolder);
void exportParaviewFlags(
    const Polyhedron& p,
    const vector<int>& vertexFlags,
    const vector<int>& edgeFlags,
    const string& subfolder
);

// dual polyhedra

Polyhedron dualPolyhedron(const Polyhedron& p);

// shortest path

void verificaVertici(const Polyhedron& poly, int id1, int id2);
void shortestPath(Polyhedron& poly, int startId, int endId);
double distanza(const Vertex& v1, const Vertex& v2);
vector<int> getVertexShortPathFlags(const Polyhedron& poly);
vector<int> getEdgeShortPathFlags(const Polyhedron& poly);
void stampaShortPathInfo(const Polyhedron& poly);