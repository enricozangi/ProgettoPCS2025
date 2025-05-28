#include "Utils.hpp"
#include "polyhedron_library.hpp"
#include <cmath>

using namespace std;
using namespace polyhedron_library;

string controllaQuadrupla(const std::vector<int>& quadrupla)
{
    if (quadrupla.size() != 4)
        return "La quadrupla non ha 4 numeri";

    for (int num : quadrupla) {
        if (num < 0)
            return "La quadrupla contiene numeri negativi";
    }

    int p = quadrupla[0];
    int q = quadrupla[1];

    if (p < 3 || q < 3)
        return "Il primo (p) o il secondo (q) numero è minore di 3";
    if (p > 3)
        return "La quadrupla va bene in teoria, ma questo programma non lo sa gestire (p > 3)";
    bool terzoZero = (quadrupla[2] == 0);
    bool quartoZero = (quadrupla[3] == 0);

    if (terzoZero && quartoZero)
        return "Sia il terzo che il quarto numero sono zero";
    if (!terzoZero && !quartoZero)
        return "Nessuno zero tra terzo e quarto numero";

    return "OK";
}
Polyhedron tetraedro() {
    Polyhedron p;
    p.id = 1;

    p.vertices = {
        {0, 1, 0, -1.0 / sqrt(2), false},
        {1, -1, 0, -1.0 / sqrt(2), false},
        {2, 0, 1, 1.0 / sqrt(2), false},
        {3, 0, -1, 1.0 / sqrt(2), false}
    };

    p.edges = {
        {0, 0, 1, false},
        {1, 1, 2, false},
        {2, 2, 0, false},
        {3, 0, 3, false},
        {4, 1, 3, false},
        {5, 2, 3, false}
    };

    p.faces = {
        {0, {p.vertices[0], p.vertices[1], p.vertices[2]}, {p.edges[0], p.edges[1], p.edges[2]}},
        {1, {p.vertices[0], p.vertices[1], p.vertices[3]}, {p.edges[0], p.edges[4], p.edges[3]}},
        {2, {p.vertices[1], p.vertices[2], p.vertices[3]}, {p.edges[1], p.edges[5], p.edges[4]}},
        {3, {p.vertices[2], p.vertices[0], p.vertices[3]}, {p.edges[2], p.edges[3], p.edges[5]}}
    };

    return p;
}

Polyhedron ottaedro() {
    Polyhedron p;
    p.id = 1;

    p.vertices = {
        {0, 1, 0, 0, false},
        {1, -1, 0, 0, false},
        {2, 0, 1, 0, false},
        {3, 0, -1, 0, false},
        {4, 0, 0, 1, false},
        {5, 0, 0, -1, false}
    };

    p.edges = {
        {0, 0, 2, false},
        {1, 0, 3, false},
        {2, 0, 4, false},
        {3, 0, 5, false},
        {4, 1, 2, false},
        {5, 1, 3, false},
        {6, 1, 4, false},
        {7, 1, 5, false},
        {8, 2, 4, false},
        {9, 2, 5, false},
        {10, 3, 4, false},
        {11, 3, 5, false}
    };

    p.faces = {
        {0, {p.vertices[0], p.vertices[2], p.vertices[4]}, {p.edges[0], p.edges[8], p.edges[2]}},
        {1, {p.vertices[0], p.vertices[4], p.vertices[3]}, {p.edges[2], p.edges[10], p.edges[1]}},
        {2, {p.vertices[0], p.vertices[3], p.vertices[5]}, {p.edges[1], p.edges[11], p.edges[3]}},
        {3, {p.vertices[0], p.vertices[5], p.vertices[2]}, {p.edges[3], p.edges[9], p.edges[0]}},
        {4, {p.vertices[1], p.vertices[2], p.vertices[4]}, {p.edges[4], p.edges[8], p.edges[6]}},
        {5, {p.vertices[1], p.vertices[4], p.vertices[3]}, {p.edges[6], p.edges[10], p.edges[5]}},
        {6, {p.vertices[1], p.vertices[3], p.vertices[5]}, {p.edges[5], p.edges[11], p.edges[7]}},
        {7, {p.vertices[1], p.vertices[5], p.vertices[2]}, {p.edges[7], p.edges[9], p.edges[4]}}
    };

    return p;
}

Polyhedron icosaedro()
{
    Polyhedron p;
    p.id = 2;

    const double phi = (1.0 + sqrt(5.0)) / 2.0;
    const double inv_norm = 1.0 / sqrt(1 + phi * phi);

    p.vertices = {
        {0, -1 * inv_norm,  phi * inv_norm, 0, false},
        {1,  1 * inv_norm,  phi * inv_norm, 0, false},
        {2, -1 * inv_norm, -phi * inv_norm, 0, false},
        {3,  1 * inv_norm, -phi * inv_norm, 0, false},
        {4, 0, -1 * inv_norm,  phi * inv_norm, false},
        {5, 0,  1 * inv_norm,  phi * inv_norm, false},
        {6, 0, -1 * inv_norm, -phi * inv_norm, false},
        {7, 0,  1 * inv_norm, -phi * inv_norm, false},
        {8,  phi * inv_norm, 0, -1 * inv_norm, false},
        {9,  phi * inv_norm, 0,  1 * inv_norm, false},
        {10, -phi * inv_norm, 0, -1 * inv_norm, false},
        {11, -phi * inv_norm, 0,  1 * inv_norm, false}
    };

    p.edges = {
        {0, 0, 1, false}, {1, 0, 5, false}, {2, 0, 7, false}, {3, 0, 10, false}, {4, 0, 11, false},
        {5, 1, 5, false}, {6, 1, 7, false}, {7, 1, 8, false}, {8, 1, 9, false}, {9, 2, 3, false},
        {10, 2, 4, false}, {11, 2, 6, false}, {12, 2, 10, false}, {13, 2, 11, false}, {14, 3, 4, false},
        {15, 3, 6, false}, {16, 3, 8, false}, {17, 3, 9, false}, {18, 4, 6, false}, {19, 4, 10, false},
        {20, 5, 9, false}, {21, 5, 11, false}, {22, 6, 10, false}, {23, 6, 11, false}, {24, 7, 8, false},
        {25, 7, 9, false}, {26, 8, 9, false}, {27, 10, 11, false}
    };

    p.faces = {
        {0, {p.vertices[0], p.vertices[1], p.vertices[5]}, {p.edges[0], p.edges[5], p.edges[1]}},
        {1, {p.vertices[0], p.vertices[5], p.vertices[11]}, {p.edges[1], p.edges[21], p.edges[4]}},
        {2, {p.vertices[0], p.vertices[11], p.vertices[10]}, {p.edges[4], p.edges[27], p.edges[3]}},
        {3, {p.vertices[0], p.vertices[10], p.vertices[2]}, {p.edges[3], p.edges[12], p.edges[12]}},
        {4, {p.vertices[0], p.vertices[2], p.vertices[7]}, {p.edges[12], p.edges[2], p.edges[2]}},
        {5, {p.vertices[0], p.vertices[7], p.vertices[1]}, {p.edges[2], p.edges[6], p.edges[0]}},
        {6, {p.vertices[1], p.vertices[7], p.vertices[8]}, {p.edges[6], p.edges[24], p.edges[7]}},
        {7, {p.vertices[1], p.vertices[8], p.vertices[9]}, {p.edges[7], p.edges[26], p.edges[8]}},
        {8, {p.vertices[1], p.vertices[9], p.vertices[5]}, {p.edges[8], p.edges[20], p.edges[5]}},
        {9, {p.vertices[2], p.vertices[10], p.vertices[4]}, {p.edges[12], p.edges[19], p.edges[10]}},
        {10, {p.vertices[2], p.vertices[4], p.vertices[6]}, {p.edges[10], p.edges[18], p.edges[11]}},
        {11, {p.vertices[2], p.vertices[6], p.vertices[3]}, {p.edges[11], p.edges[15], p.edges[9]}},
        {12, {p.vertices[2], p.vertices[3], p.vertices[8]}, {p.edges[9], p.edges[16], p.edges[7]}},
        {13, {p.vertices[2], p.vertices[8], p.vertices[7]}, {p.edges[7], p.edges[24], p.edges[2]}},
        {14, {p.vertices[3], p.vertices[6], p.vertices[4]}, {p.edges[15], p.edges[18], p.edges[14]}},
        {15, {p.vertices[3], p.vertices[4], p.vertices[9]}, {p.edges[14], p.edges[19], p.edges[17]}},
        {16, {p.vertices[3], p.vertices[9], p.vertices[8]}, {p.edges[17], p.edges[26], p.edges[16]}},
        {17, {p.vertices[4], p.vertices[10], p.vertices[11]}, {p.edges[19], p.edges[27], p.edges[13]}},
        {18, {p.vertices[4], p.vertices[11], p.vertices[5]}, {p.edges[13], p.edges[21], p.edges[1]}},
        {19, {p.vertices[4], p.vertices[5], p.vertices[9]}, {p.edges[1], p.edges[20], p.edges[15]}}
    };

    return p;
}

// function that computes the distance of a vertex from origin

double norm (const Vertex& v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}


// function that normalize vertices

Vertex normalize(const Vertex& v)
{
    double norm_v = norm(v);

    if(norm_v == 0)
    {
        cerr << "norm is 0" << endl;
        return v;
    }

    return {v.id, v.x / norm_v, v.y / norm_v, v.z / norm_v, false};
};
