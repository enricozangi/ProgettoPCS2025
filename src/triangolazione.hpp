#pragma once

#include <vector>
#include <cmath>
#include <map>
#include "polyhedron_library.hpp"

using namespace std;
using namespace polyhedron_library;

namespace polyhedron_library {

// Confronta due vertici con tolleranza
bool arePointsEqual(const Vertex& v1, const Vertex& v2, double epsilon = 1e-8) {
    return fabs(v1.x - v2.x) < epsilon &&
           fabs(v1.y - v2.y) < epsilon &&
           fabs(v1.z - v2.z) < epsilon;
}

// Calcola la distanza euclidea tra due vertici
double distance(const Vertex& v1, const Vertex& v2) {
    return sqrt(pow(v1.x - v2.x, 2) +
                pow(v1.y - v2.y, 2) +
                pow(v1.z - v2.z, 2));
}

// Inserisce un nuovo vertice, evitando duplicati
int insertVertex(vector<Vertex>& vertices, double x, double y, double z, int& nextVertexId) {
    Vertex newV{ -1, x, y, z, false };

    for (const auto& v : vertices) {
        if (arePointsEqual(v, newV)) {
            return v.id;
        }
    }

    int id = nextVertexId++;
    vertices.push_back({ id, x, y, z, false });
    return id;
}

// Funzione principale
Polyhedron suddividiTutteLeFacce(const Polyhedron& poly, int b) {
    Polyhedron newPoly;
    newPoly.id = poly.id;

    int nextVertexId = 0;
    int nextEdgeId = 0;
    double epsilon = 1e-8;

    // Copia i vertici originali
    for (const auto& v : poly.vertices) {
        newPoly.vertices.push_back({ nextVertexId++, v.x, v.y, v.z, false });
    }

    // Calcola passo p (usiamo primo edge A-B)
    const Vertex& A = poly.vertices[0];
    const Vertex& B = poly.vertices[1];
    double p = distance(A, B) / b;

    // Mappa per evitare duplicati (chiave = pair(min, max), valore = bool)
    map<pair<int, int>, bool> edgeMap;

    // Ciclo sulle facce
    for (const auto& face : poly.faces) {
        const Vertex& A = poly.vertices[face.vertices[0].id];
        const Vertex& B = poly.vertices[face.vertices[1].id];
        const Vertex& C = poly.vertices[face.vertices[2].id];

        vector<int> faceVertexIds;

        // Genera nuovi vertici sulla faccia
        for (int i = 0; i <= b; ++i) {
            for (int j = 0; j <= b - i; ++j) {
                double u = double(i) / b;
                double v = double(j) / b;
                double w = 1.0 - u - v;

                double x = u * A.x + v * B.x + w * C.x;
                double y = u * A.y + v * B.y + w * C.y;
                double z = u * A.z + v * B.z + w * C.z;

                int vid = insertVertex(newPoly.vertices, x, y, z, nextVertexId);
                faceVertexIds.push_back(vid);
            }
        }

        // Genera edges locali sulla faccia
        for (size_t i = 0; i < faceVertexIds.size(); ++i) {
            for (size_t j = i + 1; j < faceVertexIds.size(); ++j) {
                int vi = faceVertexIds[i];
                int vj = faceVertexIds[j];

                double d = distance(newPoly.vertices[vi], newPoly.vertices[vj]);
                if (fabs(d - p) < epsilon) {
                    int min_id = min(vi, vj);
                    int max_id = max(vi, vj);

                    // Aggiungi edge solo se non già presente
                    if (edgeMap.find({min_id, max_id}) == edgeMap.end()) {
                        newPoly.edges.push_back({ nextEdgeId++, min_id, max_id, false });
                        edgeMap[{min_id, max_id}] = true;
                    }
                }
            }
        }
    }

    return newPoly;
}

} // namespace polyhedron_library
