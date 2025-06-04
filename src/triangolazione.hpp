#pragma once

#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include "polyhedron_library.hpp"

using namespace std;
using namespace polyhedron_library;

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

// Crea una chiave ordinata per la faccia
string faceKey(int v1, int v2, int v3) {
    vector<int> ids = {v1, v2, v3};
    sort(ids.begin(), ids.end());
    return to_string(ids[0]) + "-" + to_string(ids[1]) + "-" + to_string(ids[2]);
}

// Funzione principale
Polyhedron triangolazione(const Polyhedron& poly, int b) {
    Polyhedron newPoly;
    newPoly.id = poly.id;

    int nextVertexId = 0;
    int nextEdgeId = 0;
    int nextFaceId = 0;
    double epsilon = 1e-8;

    map<pair<int, int>, bool> edgeMap;
    map<string, bool> faceMap;

    // Copia i vertici originali
    for (const auto& v : poly.vertices) {
        newPoly.vertices.push_back({ nextVertexId++, v.x, v.y, v.z, false });
    }
    

    // Ciclo sulle facce
    for (const auto& face : poly.faces) {
        const Vertex& A = poly.vertices[face.vertices[0].id];
        const Vertex& B = poly.vertices[face.vertices[1].id];
        const Vertex& C = poly.vertices[face.vertices[2].id];
        double p = distance(A, B) / b;
        vector<int> faceVertexIds;
        vector<Edge> localEdges;  // svuotato a ogni faccia

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

        // Genera edges locali sulla faccia e li registra anche localmente
        for (size_t i = 0; i < faceVertexIds.size(); ++i) {
            for (size_t j = i + 1; j < faceVertexIds.size(); ++j) {
                int vi = faceVertexIds[i];
                int vj = faceVertexIds[j];

                double d = distance(newPoly.vertices[vi], newPoly.vertices[vj]);
                if (fabs(d - p) < epsilon) {
                    int min_id = min(vi, vj);
                    int max_id = max(vi, vj);

                    if (edgeMap.find({min_id, max_id}) == edgeMap.end()) {
                        newPoly.edges.push_back({ nextEdgeId, min_id, max_id, false });
                        edgeMap[{min_id, max_id}] = true;
                        nextEdgeId++;
                    }

                    localEdges.push_back({ -1, min_id, max_id, false });  // ID -1: solo per confronto locale
                }
            }
        }

        // Trova le facce locali combinando 3 edges
        for (size_t i = 0; i < localEdges.size(); ++i) {
            for (size_t j = i + 1; j < localEdges.size(); ++j) {
                for (size_t k = j + 1; k < localEdges.size(); ++k) {
                    const Edge& e1 = localEdges[i];
                    const Edge& e2 = localEdges[j];
                    const Edge& e3 = localEdges[k];

                    map<int, int> vertexCount;
                    vertexCount[e1.origin]++;
                    vertexCount[e1.end]++;
                    vertexCount[e2.origin]++;
                    vertexCount[e2.end]++;
                    vertexCount[e3.origin]++;
                    vertexCount[e3.end]++;

                    vector<int> faceVertices;
                    for (const auto& [vid, count] : vertexCount) {
                        if (count == 2) {
                            faceVertices.push_back(vid);
                        }
                    }

                    if (faceVertices.size() == 3) {
                        string key = faceKey(faceVertices[0], faceVertices[1], faceVertices[2]);

                        if (faceMap.find(key) == faceMap.end()) {
                            newPoly.faces.push_back({
                                nextFaceId++,
                                { newPoly.vertices[faceVertices[0]],
                                  newPoly.vertices[faceVertices[1]],
                                  newPoly.vertices[faceVertices[2]] },
                                { e1, e2, e3 }
                            });
                            faceMap[key] = true;
                        }
                    }
                }
            }
        }
    }

    return newPoly;
}
