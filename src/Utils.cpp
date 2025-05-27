#include "Utils.hpp"
#include "polyhedron_library.hpp"
#include <cmath>
using namespace std;
string controllaQuadrupla(const std::vector<int>& quadrupla) {
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
        {0, {0, 1, 2}, {p.edges[0], p.edges[1], p.edges[2]}},
        {1, {0, 1, 3}, {p.edges[0], p.edges[4], p.edges[3]}},
        {2, {1, 2, 3}, {p.edges[1], p.edges[5], p.edges[4]}},
        {3, {2, 0, 3}, {p.edges[2], p.edges[3], p.edges[5]}}
    };

    return p;
}
Polyhedron ottaedro() {
    Polyhedron p;
    p.id = 1;

    p.vertices = {
        {0, 1, 0, 0, false},   // +X
        {1, -1, 0, 0, false},  // -X
        {2, 0, 1, 0, false},   // +Y
        {3, 0, -1, 0, false},  // -Y
        {4, 0, 0, 1, false},   // +Z
        {5, 0, 0, -1, false}   // -Z
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
        {0, {0, 2, 4}, {p.edges[0], p.edges[8], p.edges[2]}},
        {1, {0, 4, 3}, {p.edges[2], p.edges[10], p.edges[1]}},
        {2, {0, 3, 5}, {p.edges[1], p.edges[11], p.edges[3]}},
        {3, {0, 5, 2}, {p.edges[3], p.edges[9], p.edges[0]}},
        {4, {1, 2, 4}, {p.edges[4], p.edges[8], p.edges[6]}},
        {5, {1, 4, 3}, {p.edges[6], p.edges[10], p.edges[5]}},
        {6, {1, 3, 5}, {p.edges[5], p.edges[11], p.edges[7]}},
        {7, {1, 5, 2}, {p.edges[7], p.edges[9], p.edges[4]}}
    };

    return p;
}
Polyhedron icosaedro() {
    Polyhedron p;
    p.id = 2;

    const double phi = (1.0 + sqrt(5.0)) / 2.0;
    const double inv_norm = 1.0 / sqrt(1 + phi * phi);

    p.vertices = {
        {0, 0, 1 * inv_norm, phi * inv_norm, false},
        {1, 0, -1 * inv_norm, phi * inv_norm, false},
        {2, 0, 1 * inv_norm, -phi * inv_norm, false},
        {3, 0, -1 * inv_norm, -phi * inv_norm, false},
        {4, 1 * inv_norm, phi * inv_norm, 0, false},
        {5, -1 * inv_norm, phi * inv_norm, 0, false},
        {6, 1 * inv_norm, -phi * inv_norm, 0, false},
        {7, -1 * inv_norm, -phi * inv_norm, 0, false},
        {8, phi * inv_norm, 0, 1 * inv_norm, false},
        {9, -phi * inv_norm, 0, 1 * inv_norm, false},
        {10, phi * inv_norm, 0, -1 * inv_norm, false},
        {11, -phi * inv_norm, 0, -1 * inv_norm, false}
    };

    p.edges = {
        {0, 0, 1, false}, {1, 0, 4, false}, {2, 0, 5, false}, {3, 0, 8, false}, {4, 0, 9, false},
        {5, 1, 4, false}, {6, 1, 5, false}, {7, 1, 8, false}, {8, 1, 9, false}, {9, 2, 3, false},
        {10, 2, 4, false}, {11, 2, 5, false}, {12, 2, 10, false}, {13, 2, 11, false}, {14, 3, 4, false},
        {15, 3, 5, false}, {16, 3, 10, false}, {17, 3, 11, false}, {18, 4, 8, false}, {19, 4, 10, false},
        {20, 5, 9, false}, {21, 5, 11, false}, {22, 6, 7, false}, {23, 6, 8, false}, {24, 6, 9, false},
        {25, 6, 10, false}, {26, 6, 11, false}, {27, 7, 8, false}, {28, 7, 9, false}, {29, 7, 10, false}
    };

    p.faces = {
        {0, {0,1,4}, {p.edges[0], p.edges[1], p.edges[5]}},
        {1, {0,4,5}, {p.edges[1], p.edges[2], p.edges[18]}},
        {2, {0,5,8}, {p.edges[2], p.edges[3], p.edges[20]}},
        {3, {0,8,9}, {p.edges[3], p.edges[4], p.edges[7]}},
        {4, {0,9,1}, {p.edges[4], p.edges[8], p.edges[0]}},
        {5, {1,4,8}, {p.edges[5], p.edges[7], p.edges[18]}},
        {6, {1,5,9}, {p.edges[6], p.edges[8], p.edges[20]}},
        {7, {1,8,10}, {p.edges[7], p.edges[19], p.edges[12]}},
        {8, {1,9,11}, {p.edges[8], p.edges[21], p.edges[13]}},
        {9, {2,3,4}, {p.edges[9], p.edges[14], p.edges[10]}},
        {10, {2,4,5}, {p.edges[10], p.edges[11], p.edges[18]}},
        {11, {2,5,10}, {p.edges[11], p.edges[12], p.edges[19]}},
        {12, {2,10,11}, {p.edges[12], p.edges[13], p.edges[16]}},
        {13, {2,11,3}, {p.edges[13], p.edges[17], p.edges[9]}},
        {14, {3,4,10}, {p.edges[14], p.edges[16], p.edges[19]}},
        {15, {3,5,11}, {p.edges[15], p.edges[17], p.edges[21]}},
        {16, {3,10,6}, {p.edges[16], p.edges[25], p.edges[22]}},
        {17, {3,11,7}, {p.edges[17], p.edges[26], p.edges[22]}},
        {18, {4,8,10}, {p.edges[18], p.edges[19], p.edges[23]}},
        {19, {5,9,11}, {p.edges[20], p.edges[21], p.edges[28]}}
    };

    return p;
}