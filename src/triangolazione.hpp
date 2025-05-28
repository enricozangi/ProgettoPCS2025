#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include "polyhedron_library.hpp"
#include "Utils.hpp"
using namespace std;

namespace polyhedron_library {

int insertVertex(vector<Vertex>& newVertices, map<vector<double>, int>& vertexMap,
                 double x, double y, double z, int& nextVertexId) {
    vector<double> key = { x, y, z };
    if (vertexMap.find(key) != vertexMap.end()) {
        return vertexMap[key];
    }
    else {
        int id = nextVertexId++;
        newVertices.push_back({ id, x, y, z, false });
        vertexMap[key] = id;
        return id;
    }
}

Polyhedron suddividiTutteLeFacce(Polyhedron poly, int b) {
    int nextVertexId = poly.numVertices();
    int nextEdgeId = poly.numEdges();

    map<vector<double>, int> vertexMap;
    for (const auto& v : poly.vertices) {
        vertexMap[{v.x, v.y, v.z}] = v.id;
    }

    vector<Vertex> newVertices = poly.vertices;
    vector<Edge> newEdges = poly.edges;

    for (const Face& face : poly.faces) {
        Vertex A = face.vertices[0];
        Vertex B = face.vertices[1];
        Vertex C = face.vertices[2];

        for (int i = 0; i <= b; ++i) {
            for (int j = 0; j <= b - i; ++j) {
                double u = double(i) / b;
                double v = double(j) / b;
                double w = 1.0 - u - v;

                double x = u * A.x + v * B.x + w * C.x;
                double y = u * A.y + v * B.y + w * C.y;
                double z = u * A.z + v * B.z + w * C.z;

                insertVertex(newVertices, vertexMap, x, y, z, nextVertexId);
            }
        }

        for (int i = 0; i <= b; ++i) {
            for (int j = 0; j <= b - i; ++j) {
                int id1 = insertVertex(newVertices, vertexMap,
                    (double(i) / b) * A.x + (double(j) / b) * B.x + (1.0 - double(i)/b - double(j)/b) * C.x,
                    (double(i) / b) * A.y + (double(j) / b) * B.y + (1.0 - double(i)/b - double(j)/b) * C.y,
                    (double(i) / b) * A.z + (double(j) / b) * B.z + (1.0 - double(i)/b - double(j)/b) * C.z,
                    nextVertexId
                );

                if (i < b) { // questo ciclo collega il vertice con quello alla sua destra se i<b
                    int id2 = insertVertex(newVertices, vertexMap,
                        (double(i + 1) / b) * A.x + (double(j) / b) * B.x + (1.0 - double(i + 1)/b - double(j)/b) * C.x,
                        (double(i + 1) / b) * A.y + (double(j) / b) * B.y + (1.0 - double(i + 1)/b - double(j)/b) * C.y,
                        (double(i + 1) / b) * A.z + (double(j) / b) * B.z + (1.0 - double(i + 1)/b - double(j)/b) * C.z,
                        nextVertexId
                    );
                    newEdges.push_back({ nextEdgeId++, id1, id2, false });
                }
                if (j < b) { // questo if collega il vertice con quello in obliquo a sinistra
                    int id3 = insertVertex(newVertices, vertexMap,
                        (double(i) / b) * A.x + (double(j + 1) / b) * B.x + (1.0 - double(i)/b - double(j + 1)/b) * C.x,
                        (double(i) / b) * A.y + (double(j + 1) / b) * B.y + (1.0 - double(i)/b - double(j + 1)/b) * C.y,
                        (double(i) / b) * A.z + (double(j + 1) / b) * B.z + (1.0 - double(i)/b - double(j + 1)/b) * C.z,
                        nextVertexId
                    );
                    newEdges.push_back({ nextEdgeId++, id1, id3, false });
                }

                if (i < b && j < b) { //questo if collega il vertice con quello in obliquo a destra
                    int id4 = insertVertex(newVertices, vertexMap,
                        (double(i + 1) / b) * A.x + (double(j + 1) / b) * B.x + (1.0 - double(i + 1)/b - double(j + 1)/b) * C.x,
                        (double(i + 1) / b) * A.y + (double(j + 1) / b) * B.y + (1.0 - double(i + 1)/b - double(j + 1)/b) * C.y,
                        (double(i + 1) / b) * A.z + (double(j + 1) / b) * B.z + (1.0 - double(i + 1)/b - double(j + 1)/b) * C.z,
                        nextVertexId
                    );
                    newEdges.push_back({ nextEdgeId++, id1, id4, false });
                }
            }
        }
    }

    poly.vertices = newVertices;
    poly.edges = newEdges;

    return poly;
}

}  // namespace polyhedron_library

