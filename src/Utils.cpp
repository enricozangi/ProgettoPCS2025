#include <iostream>
#include <fstream>
#include <cmath>

#include "Utils.hpp"
#include "polyhedron_library.hpp"

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

Polyhedron ottaedro()
{
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
	Polyhedron P;
	P.id = 2;

	using std::numbers::phi;

	P.vertices = {
        {0, -1, phi, 0, false},
        {1, 1, phi, 0, false},
        {2, -1, -phi, 0, false},
        {3, 1, -phi, 0, false},
        {4, 0, -1, phi, false},
        {5, 0, 1, phi, false},
        {6, 0, -1, -phi, false},
        {7, 0, 1, -phi, false},
        {8, phi, 0, -1, false},
        {9, phi, 0, 1, false},
        {10, -phi, 0, -1, false},
        {11, -phi, 0, 1, false}
    };

    for (Vertex& v : P.vertices) {
        normalize(v);
    }

    P.edges = {
        {0, 0, 5, false}, {1, 0, 1, false}, {2, 0, 11, false}, {3, 0, 10, false}, {4, 0, 7, false},
        {5, 1, 5, false}, {6, 1, 7, false}, {7, 1, 8, false}, {8, 1, 9, false}, {9, 5, 9, false},
        {10, 5, 11, false}, {11, 9, 8, false}, {12, 8, 7, false}, {13, 9, 4, false}, {14, 9, 3, false},
        {15, 4, 3, false}, {16, 5, 4, false}, {17, 4, 11, false}, {18, 2, 4, false}, {19, 2, 11, false},
        {20, 2, 3, false}, {21, 2, 6, false}, {22, 6, 3, false}, {23, 8, 3, false}, {24, 8, 6, false},
        {25, 7, 6, false}, {26, 2, 10, false}, {27, 10, 6, false}, {28, 10, 7, false}, {29, 11, 10, false}
    };

	// Set values of faces
	P.faces = {
		{0, {P.vertices[5], P.vertices[0], P.vertices[11]}, {P.edges[0], P.edges[2], P.edges[10]}},
		{1, {P.vertices[0], P.vertices[5], P.vertices[1]}, {P.edges[0], P.edges[5], P.edges[1]}},
		{2, {P.vertices[5], P.vertices[1], P.vertices[9]}, {P.edges[5], P.edges[8], P.edges[9]}},
		{3, {P.vertices[5], P.vertices[4], P.vertices[9]}, {P.edges[16], P.edges[13], P.edges[9]}},
		{4, {P.vertices[4], P.vertices[5], P.vertices[11]}, {P.edges[16], P.edges[10], P.edges[17]}},
		{5, {P.vertices[4], P.vertices[3], P.vertices[9]}, {P.edges[15], P.edges[14], P.edges[13]}},
		{6, {P.vertices[4], P.vertices[11], P.vertices[2]}, {P.edges[17], P.edges[19], P.edges[18]}},
		{7, {P.vertices[2], P.vertices[3], P.vertices[6]}, {P.edges[20], P.edges[22], P.edges[21]}},
		{8, {P.vertices[6], P.vertices[2], P.vertices[10]}, {P.edges[21], P.edges[26], P.edges[27]}},
		{9, {P.vertices[3], P.vertices[8], P.vertices[6]}, {P.edges[23], P.edges[24], P.edges[22]}},
		{10, {P.vertices[8], P.vertices[9], P.vertices[3]}, {P.edges[11], P.edges[14], P.edges[23]}},
		{11, {P.vertices[9], P.vertices[8], P.vertices[1]}, {P.edges[11], P.edges[7], P.edges[8]}},
		{12, {P.vertices[8], P.vertices[7], P.vertices[1]}, {P.edges[12], P.edges[6], P.edges[7]}},
		{13, {P.vertices[6], P.vertices[7], P.vertices[8]}, {P.edges[25], P.edges[12], P.edges[24]}},
		{14, {P.vertices[7], P.vertices[10], P.vertices[6]}, {P.edges[28], P.edges[27], P.edges[25]}},
		{15, {P.vertices[7], P.vertices[10], P.vertices[0]}, {P.edges[28], P.edges[3], P.edges[4]}},
		{16, {P.vertices[0], P.vertices[10], P.vertices[11]}, {P.edges[3], P.edges[29], P.edges[2]}},
		{17, {P.vertices[4], P.vertices[2], P.vertices[3]}, {P.edges[18], P.edges[20], P.edges[15]}},
		{18, {P.vertices[1], P.vertices[0], P.vertices[7]}, {P.edges[1], P.edges[4], P.edges[6]}},
		{19, {P.vertices[11], P.vertices[2], P.vertices[10]}, {P.edges[19], P.edges[26], P.edges[29]}}
	};

	return P;
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

// Cell0Ds.txt (vertices)

void exportVertices(const vector<Vertex>& vertices)
{

    ofstream outFile("Cell0Ds.txt");

    if (!outFile)
    {
        cerr << "file not found" << endl;
        return;
    }

    for(const Vertex& v : vertices)
    {
        outFile << v.id << ";" << v.x << ";" << v.y << ";" << v.z << ";\n";
    }


    outFile.close();
}

    // Cell1Ds.txt (edges)

void exportEdges(const vector<Edge>& edges)
{

    ofstream outFile("Cell1Ds.txt");

    if (!outFile)
    {
        cerr << "file not found" << endl;
        return;
    }

    for(const Edge& e : edges)
    {
        outFile << e.id << ";" << e.origin << ";" << e.end << ";\n";
    }

    outFile.close();
}

    // Cell2Ds.txt (faces)

void exportFaces(const vector<Face>& faces)
{
    ofstream outFile("Cell2Ds.txt");

    if (!outFile)
    {
        cerr << "file not found" << endl;
        return;
    }

    for(const Face& f : faces)
    {
        outFile << f.id << ";";
        for (size_t i = 0; i < f.vertices.size(); ++i)
        {
            outFile << f.vertices[i].id;
            if (i < f.vertices.size() - 1) outFile << ",";
        }
        outFile << ";";

        for (size_t i = 0; i < f.edges.size(); ++i)
        {
            outFile << f.edges[i].id;
            if (i < f.edges.size() - 1) outFile << ",";
        }
        outFile << ";\n";
    }

    outFile.close();
}

    // Cell3Ds.txt (polyhedra)

void exportPolyhedra(const Polyhedron& p)
{
    ofstream outFile("Cell3Ds.txt");

    if (!outFile)
    {
        cerr << "file not found" << endl;
        return;
    }

    outFile << p.id << ";";

    for (size_t i = 0; i < p.vertices.size(); ++i)
    {
        outFile << p.vertices[i].id;
        if (i < p.vertices.size() - 1) outFile << ",";
    }
    outFile << ";";

    for (size_t i = 0; i < p.edges.size(); ++i)
    {
        outFile << p.edges[i].id;
        if (i < p.edges.size() - 1) outFile << ",";
    }
    outFile << ";";

    for (size_t i = 0; i < p.faces.size(); ++i)
    {
        outFile << p.faces[i].id;
        if (i < p.faces.size() - 1) outFile << ",";
    }
    outFile << ";\n";

    outFile.close();
}
