#pragma once
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include "polyhedron_library.hpp"
#include "Utils.hpp"

using namespace std;
using namespace polyhedron_library;
Vertex faceCentroid1(const Face& f, int id)
{
    double x = 0, y = 0, z = 0;
    for (const auto& v : f.vertices) {
        x += v.x;
        y += v.y;
        z += v.z;
    }
    int n = f.vertices.size();
    return Vertex{id, x / n, y / n, z / n, false};
}
// Confronta due vertici con tolleranza
bool arePointsEqual(const Vertex& v1, const Vertex& v2, double epsilon = 1e-8) {
    return fabs(v1.x - v2.x) < epsilon &&
           fabs(v1.y - v2.y) < epsilon &&
           fabs(v1.z - v2.z) < epsilon;
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

    map<pair<int, int>, int> edgeMap;
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
        double p = distanza(A, B) / b;
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

                double d = distanza(newPoly.vertices[vi], newPoly.vertices[vj]);
                if (fabs(d - p) < epsilon) {
                    int min_id = min(vi, vj);
                    int max_id = max(vi, vj);

                    if (edgeMap.find({min_id, max_id}) == edgeMap.end()) {
                        newPoly.edges.push_back({ nextEdgeId, min_id, max_id, false });
                        edgeMap[{min_id, max_id}] = nextEdgeId;
                        nextEdgeId++;
                    }                                    
                    localEdges.push_back({ edgeMap[{min_id, max_id}], min_id, max_id, false });
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
int mid(const vector<int>& v) {
    vector<int> tmp = v;
    sort(tmp.begin(), tmp.end());
    return tmp[1];
}
Polyhedron triangolazione2(const Polyhedron& poly, int b) {
    Polyhedron newPoly;
    newPoly.id = poly.id;
    int nextVertexId = 0, nextEdgeId = 0, nextFaceId = 0;
    double epsilon = 1e-8;

    Polyhedron poly1 = triangolazione(poly, b);
    Polyhedron poly2 = triangolazione(poly, 2 * b);

    for (const auto& face : poly.faces) {
        vector<int> localVertices;
        vector<int> verticiSulLato;
        vector<Edge> localEdges;

        const Vertex& A = poly.vertices[face.vertices[0].id];
        const Vertex& B = poly.vertices[face.vertices[1].id];
        const Vertex& C = poly.vertices[face.vertices[2].id];

        double ux = B.x - A.x, uy = B.y - A.y, uz = B.z - A.z;
        double vx = C.x - A.x, vy = C.y - A.y, vz = C.z - A.z;
        double nx = uy * vz - uz * vy;
        double ny = uz * vx - ux * vz;
        double nz = ux * vy - uy * vx;
        double d = -(nx * A.x + ny * A.y + nz * A.z);

        double passo = distanza(A, B) / (2.0 * b);

        vector<pair<Vertex, Vertex>> edges = {{A, B}, {B, C}, {C, A}};
        for (const auto& [v1, v2] : edges) {
            double dx = (v2.x - v1.x) / (2.0 * b);
            double dy = (v2.y - v1.y) / (2.0 * b);
            double dz = (v2.z - v1.z) / (2.0 * b);

            int prevId = -1;

            for (int i = 0; i <= 2 * b; ++i) {
                double x = v1.x + i * dx;
                double y = v1.y + i * dy;
                double z = v1.z + i * dz;
                int id = insertVertex(newPoly.vertices, x, y, z, nextVertexId);

                if (find(localVertices.begin(), localVertices.end(), id) == localVertices.end()) {
                    localVertices.push_back(id);
                }
                if (find(verticiSulLato.begin(), verticiSulLato.end(), id) == verticiSulLato.end()) {
                    verticiSulLato.push_back(id);
                }

                if (prevId != -1) {
                    int min_id = min(prevId, id);
                    int max_id = max(prevId, id);

                    // Controllo su newPoly
                    bool alreadyInNewPoly = any_of(newPoly.edges.begin(), newPoly.edges.end(), [&](const Edge& e) {
                        return (e.origin == min_id && e.end == max_id) || (e.origin == max_id && e.end == min_id);
                    });

                    if (!alreadyInNewPoly) {
                        newPoly.edges.push_back({ nextEdgeId++, min_id, max_id, false });
                    }

                    // Controllo su localEdges
                    bool alreadyInLocal = any_of(localEdges.begin(), localEdges.end(), [&](const Edge& e) {
                        return (e.origin == min_id && e.end == max_id) || (e.origin == max_id && e.end == min_id);
                    });

                    if (!alreadyInLocal) {
                        localEdges.push_back({ -1, min_id, max_id, false }); // id fittizio, se necessario
                    }
                }

                prevId = id;
            }
        }


        for (const auto& f1 : poly1.faces) {
            Vertex centro = faceCentroid1(f1, nextVertexId);
            double val = nx * centro.x + ny * centro.y + nz * centro.z + d;
            if (fabs(val) < epsilon) {
                newPoly.vertices.push_back({nextVertexId, centro.x, centro.y, centro.z, false});
                localVertices.push_back(nextVertexId);
                ++nextVertexId;
            }
        }
        for (size_t i = 0; i < localVertices.size(); ++i) {
            for (size_t j = i + 1; j < localVertices.size(); ++j) {
                int vi = localVertices[i], vj = localVertices[j];
                double d = distanza(newPoly.vertices[vi], newPoly.vertices[vj]);
                if (d < 1.5 * passo) {
                    bool esisteInPoly2 = false;
                    for (const auto& e : poly2.edges) {
                        const Vertex& a = poly2.vertices[e.origin];
                        const Vertex& b = poly2.vertices[e.end];
                        if ((arePointsEqual(newPoly.vertices[vi], a) && arePointsEqual(newPoly.vertices[vj], b)) ||
                            (arePointsEqual(newPoly.vertices[vi], b) && arePointsEqual(newPoly.vertices[vj], a))) {
                            esisteInPoly2 = true;
                            break;
                        }
                    }
                    if (!esisteInPoly2) {
                        bool giaPresente = false;
                        for (const auto& e : newPoly.edges) {
                            if ((e.origin == vi && e.end == vj) || (e.origin == vj && e.end == vi)) {
                                giaPresente = true;
                                break;
                            }
                        }
                        if (!giaPresente) {
                            newPoly.edges.push_back({nextEdgeId, vi, vj, false});
                            localEdges.push_back({nextEdgeId++, vi, vj, false});
                        }
                    }
                }
            }
        }
        for (size_t i = 0; i < localEdges.size(); ++i) {
            for (size_t j = i + 1; j < localEdges.size(); ++j) {
                for (size_t k = j + 1; k < localEdges.size(); ++k) {
                    const Edge& e1 = localEdges[i];
                    const Edge& e2 = localEdges[j];
                    const Edge& e3 = localEdges[k];

                    map<int, int> verticeCounter;
                    verticeCounter[e1.origin]++;
                    verticeCounter[e1.end]++;
                    verticeCounter[e2.origin]++;
                    verticeCounter[e2.end]++;
                    verticeCounter[e3.origin]++;
                    verticeCounter[e3.end]++;

                    // Verifica che ci siano esattamente 3 vertici, ciascuno presente due volte
                    if (verticeCounter.size() == 3 &&
                        all_of(verticeCounter.begin(), verticeCounter.end(),
                                    [](const pair<int, int>& p) { return p.second == 2; })) {
                        
                        vector<Vertex> faceVertices;
                        for (const auto& [id, count] : verticeCounter) {
                            faceVertices.push_back(newPoly.vertices[id]);
                        }

                        newPoly.faces.push_back({
                            nextFaceId++,
                            faceVertices,
                            {e1, e2, e3}
                        });
                    }
                }
            }
        }
    }
    return newPoly;
}
