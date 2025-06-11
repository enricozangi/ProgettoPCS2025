#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Eigen>
#include <queue>
#include <string>
#include <vector>
#include "UCDUtilities.hpp"
#include "Utils.hpp"
#include "polyhedron_library.hpp"

using namespace std;
using namespace polyhedron_library;

// Funzione che calcola la distanza tra due vertici

double distanza(const Vertex& v1, const Vertex& v2) {
    return sqrt(pow(v1.x - v2.x, 2) +
                pow(v1.y - v2.y, 2) +
                pow(v1.z - v2.z, 2));
}

// Funzione che controlla la quadrupla di numeri interi

string controllaQuadrupla(const vector<int>& quadrupla)
{
    if (quadrupla.size() != 4)
        return "la quadrupla non ha 4 numeri";

    for (int num : quadrupla) {
        if (num < 0)
            return "la quadrupla contiene numeri negativi";
    }

    int p = quadrupla[0];
    int q = quadrupla[1];
    int b = quadrupla[2];
    int c = quadrupla[3];

    if (p < 3 || q < 3)
        return "il primo numero (p) o il secondo numero (q) è minore di 3.";

    if (p > 3)
        return "la quadrupla inserita non è adatta a questo programma (p > 3).";

    if ((b == 0 && c == 0))
        return "i parametri b e c sono entrambi uguali a 0.";

    if ((b != 0 && c != 0) && (b != c))
        return "i parametri b e c sono diversi da zero ma non sono uguali tra loro.";

    return "OK";
}

// Funzione che crea un tetraedro regolare

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

// Funzione che crea un ottaedro regolare

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

// Funzione che crea un icosaedro regolare

Polyhedron icosaedro()
{
	Polyhedron P;
	P.id = 2;

    using numbers::phi;

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

    P.edges = {
        {0, 0, 5, false}, {1, 0, 1, false}, {2, 0, 11, false}, {3, 0, 10, false}, {4, 0, 7, false},
        {5, 1, 5, false}, {6, 1, 7, false}, {7, 1, 8, false}, {8, 1, 9, false}, {9, 5, 9, false},
        {10, 5, 11, false}, {11, 9, 8, false}, {12, 8, 7, false}, {13, 9, 4, false}, {14, 9, 3, false},
        {15, 4, 3, false}, {16, 5, 4, false}, {17, 4, 11, false}, {18, 2, 4, false}, {19, 2, 11, false},
        {20, 2, 3, false}, {21, 2, 6, false}, {22, 6, 3, false}, {23, 8, 3, false}, {24, 8, 6, false},
        {25, 7, 6, false}, {26, 2, 10, false}, {27, 10, 6, false}, {28, 10, 7, false}, {29, 11, 10, false}
    };

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

// funzione che calcola la distanza di un vertice dall'origine

double norm (const Vertex& v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}


// funzione che normalizza i vertici

void normalize(Vertex& v)
{
    double norm_v = norm(v);

    if(norm_v > 1e-8)
    {
        v.x /= norm_v;
        v.y /= norm_v; 
        v.z /= norm_v;
    }
    else
    {
        cerr << "norm = 0" << endl;
    }
};

// Funzioni per l'esportazione dei dati in file di testo
// Cell0Ds.txt (vertices)

void exportVertices(const vector<Vertex>& vertices, const string& subfolder)
{
    string path = "../ListPolygons/" + subfolder + "/Cell0Ds.txt";
    ofstream outFile(path);

    if (!outFile)
    {
        cerr << "file not found" << endl;
        return;
    }

    outFile << "id;x;y;z;\n";

    for(const Vertex& v : vertices)
    {
        outFile << v.id << ";" << v.x << ";" << v.y << ";" << v.z << ";\n";
    }


    outFile.close();
}

    // Cell1Ds.txt (edges)

void exportEdges(const vector<Edge>& edges, const string& subfolder)
{

    string path = "../ListPolygons/" + subfolder + "/Cell1Ds.txt";
    ofstream outFile(path);
    if (!outFile)
    {
        cerr << "file not found" << endl;
        return;
    }

    outFile << "id;origin;end;\n";

    for(const Edge& e : edges)
    {
        outFile << e.id << ";" << e.origin << ";" << e.end << ";\n";
    }

    outFile.close();
}

    // Cell2Ds.txt (faces)

void exportFaces(const vector<Face>& faces, const string& subfolder)
{
    string path = "../ListPolygons/" + subfolder + "/Cell2Ds.txt";
    ofstream outFile(path);
    if (!outFile)
    {
        cerr << "file not found" << endl;
        return;
    }

    outFile << "id;vertices;edges;\n";

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

void exportPolyhedra(const Polyhedron& p, const string& subfolder)
{
    string path = "../ListPolygons/" + subfolder + "/Cell3Ds.txt";
    ofstream outFile(path);
    if (!outFile)
    {
        cerr << "file not found" << endl;
        return;
    }

    outFile << "id;vertices;edges;faces;\n";
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

// Export in Paraview

void exportParaview(const Polyhedron& p, const string& subfolder)
{
    string path1 = "../ListPolygons/" + subfolder + "/Cell0Ds.inp";
    string path2 = "../ListPolygons/" + subfolder + "/Cell1Ds.inp";

    Eigen::MatrixXd Cell0Ds;
    Cell0Ds = Eigen::MatrixXd::Zero(3, p.numVertices());

    for(const auto& v : p.vertices)
    {
        Cell0Ds.col(v.id) << v.x, v.y, v.z;
    }

    Eigen::MatrixXi Cell1Ds;
    Cell1Ds = Eigen::MatrixXi::Zero(2, p.numEdges());

    for(const auto& e : p.edges)
    {
        int id = e.id;

        Cell1Ds(0, id) = e.origin;
        Cell1Ds(1, id) = e.end;
    }

    Gedim::UCDUtilities utilities;
    utilities.ExportPoints(path1, Cell0Ds);

    utilities.ExportSegments(path2, Cell0Ds, Cell1Ds);

}

// Export in Paraview con visualizzazione del cammino minimo

void exportParaviewFlags(
    const Polyhedron& p,
    const vector<int>& vertexFlags,
    const vector<int>& edgeFlags,
    const string& subfolder
)
{
    string path1 = "../ListPolygons/" + subfolder + "/Cell0Ds.inp";
    string path2 = "../ListPolygons/" + subfolder + "/Cell1Ds.inp";

    Eigen::MatrixXd Cell0Ds = Eigen::MatrixXd::Zero(3, p.numVertices());
    for(const auto& v : p.vertices)
    {
        Cell0Ds.col(v.id) << v.x, v.y, v.z;
    }

    Eigen::MatrixXi Cell1Ds = Eigen::MatrixXi::Zero(2, p.numEdges());
    for(const auto& e : p.edges)
    {
        int id = e.id;
        Cell1Ds(0, id) = e.origin;
        Cell1Ds(1, id) = e.end;
    }

    vector<double> vertices_double(vertexFlags.begin(), vertexFlags.end());
    vector<double> edges_double(edgeFlags.begin(), edgeFlags.end());

    Gedim::UCDProperty<double> vflagProperty;
    Gedim::UCDProperty<double> eflagProperty;

    vflagProperty.Label = "vertexShortPath";
    vflagProperty.UnitLabel = "";
    vflagProperty.NumComponents = 1;
    vflagProperty.Data = vertices_double.data();

    eflagProperty.Label = "edgesShortPath";
    eflagProperty.UnitLabel = "";
    eflagProperty.NumComponents = 1;
    eflagProperty.Data = edges_double.data();

    vector<Gedim::UCDProperty<double>> v_properties{vflagProperty};
    vector<Gedim::UCDProperty<double>> e_properties{eflagProperty};

    Gedim::UCDUtilities utilities;
    utilities.ExportPoints(path1, Cell0Ds, v_properties); 
    utilities.ExportSegments(path2, Cell0Ds, Cell1Ds, v_properties, e_properties);
}

// Funzione che calcola il baricentro di una faccia

Vertex faceCentroid(const Face& f, int id)
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

// Funzione per verificare se due facce condividono un lato comune

bool haveCommonEdge(const Face& f1, const Face& f2)
{
    for (const auto& e1 : f1.edges) {
        for (const auto& e2 : f2.edges) {
            if (e1.id == e2.id) {
                return true;
            }
        }
    }
    return false;
}

// Funzione per trovare le facce adiacenti a una data faccia in un poliedro

vector<Face> facceAdiacenti(const Polyhedron& p, const Face& f)
{
    vector<Face> adiacenti;
    for (const auto& other : p.faces)
    {
        if (other.id != f.id)
        {
            if (haveCommonEdge(f, other))
            {
                adiacenti.push_back(other);
            }
        }
    }
    return adiacenti;
}

// Funzione per creare il poliedro duale di un dato poliedro


Polyhedron dualPolyhedron(const Polyhedron& p)
{
    Polyhedron dual;
    dual.id = p.id;

    // Crea i vertici del poliedro di Goldberg

    for(const auto& f : p.faces)
    {
        Vertex centroid = faceCentroid(f, f.id);
        dual.vertices.push_back(centroid);
    }

    // Crea gli spigoli del poliedro di Goldberg

    for(const auto& f : p.faces)
    {
        vector<Face> adj = facceAdiacenti(p, f);
        for(const auto& a : adj)
        {
            Edge e = {-1, f.id, a.id, false};
            if(f.id < a.id)
            {
                e.id = dual.edges.size();
                dual.edges.push_back(e);
            }
        }
    }
    // Crea le facce del poliedro di Goldberg

    // Ogni vertice del poliedro originale corrisponde a una faccia del duale.
    // Per ogni vertice, trova tutte le facce che lo contengono e crea una faccia del duale
    for (const auto& v : p.vertices) {
        vector<Vertex> dualFaceVertices;
        vector<Edge> dualFaceEdges;
        vector<int> faceIds;

        // Trova tutte le facce che contengono il vertice v
        for (const auto& f : p.faces) {
            for (const auto& fv : f.vertices) {
                if (fv.id == v.id) {
                    dualFaceVertices.push_back(dual.vertices[f.id]);
                    faceIds.push_back(f.id);
                    break;
                }
            }
        }   
        // Crea gli edges della faccia duale
        for (size_t i = 0; i < faceIds.size(); ++i)
        {
            for (size_t j = i + 1; j < faceIds.size(); ++j)
            {
                int from = faceIds[i];
                int to = faceIds[j];
                // Trova l'edge corrispondente
                for (const auto& e : dual.edges) {
                    if ((e.origin == from && e.end == to) || (e.origin == to && e.end == from)) {
                        dualFaceEdges.push_back(e);
                        break;
                    }
                }
            }  
        }

        Face dualFace;
        dualFace.id = dual.faces.size();
        dualFace.vertices = dualFaceVertices;
        dualFace.edges = dualFaceEdges;
        dual.faces.push_back(dualFace);
    }
    for(Vertex& v : dual.vertices)
    {
        normalize(v);
    }
    return dual;
}

// Funzione per verificare l'esistenza di due vertici in un poliedro

void verificaVertici(const Polyhedron& poly, int id1, int id2) {
    bool trovato1 = false, trovato2 = false;

    for (const Vertex& v : poly.vertices) {
        if (v.id == id1) trovato1 = true;
        if (v.id == id2) trovato2 = true;
        if (trovato1 && trovato2) break;
    }

    if (!trovato1 || !trovato2) {
        cerr << "Errore: uno o entrambi gli ID dei vertici specificati non esistono nel poliedro." << endl;
    }
}

// Funzione per calcolare il cammino minimo tra due vertici in un poliedro
// Utilizza l'algoritmo di Dijkstra per trovare il cammino più breve tra due vertici

void shortestPath(Polyhedron& poly, int id1, int id2)
{
    int n = poly.vertices.size();
    vector<double> dist(n, 1e9);
    vector<int> prev(n, -1);
    vector<bool> visited(n, false);

    dist[id1] = 0.0;

    // Min-heap: (distance, vertex id)
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    pq.push({0.0, id1});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (visited[u]) continue;
        visited[u] = true;
        if (u == id2) break;

        // For each neighbor via edges
        for (const auto& edge : poly.edges) {
            int v = -1;
            if (edge.origin == u) v = edge.end;
            else if (edge.end == u) v = edge.origin;
            else continue;

            double weight = distanza(poly.vertices[u], poly.vertices[v]);
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    if (prev[id2] == -1 && id1 != id2) {
        cerr << "Nessun percorso tra " << id1 << " e " << id2 << endl;
        return;
    }

    // Ricostruisce il percorso da id1 a id2
    vector<int> path;
    for (int at = id2; at != -1; at = prev[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    // Marca i vertici nel percorso
    for (int id : path) {
        poly.vertices[id].ShortPath = true;
    }

    // Marca i lati nel percorso
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        int from = path[i];
        int to = path[i + 1];
        for (auto& edge : poly.edges) {
            if ((edge.origin == from && edge.end == to) ||
                (edge.origin == to && edge.end == from)) {
                edge.ShortPath = true;
                break;
            }
        }
    }
}

// Funzioni per ottenere i flag dei vertici e degli spigoli dello short path

vector<int> getVertexShortPathFlags(const Polyhedron& poly) {
    vector<int> flags;
    for (const auto& v : poly.vertices) {
        flags.push_back(v.ShortPath ? 1 : 0);
    }
    return flags;
}
vector<int> getEdgeShortPathFlags(const Polyhedron& poly) {
    vector<int> flags;
    for (const auto& e : poly.edges) {
        flags.push_back(e.ShortPath ? 1 : 0);
    }
    return flags;
}

// Funzione per stampare le informazioni sul cammino minimo in un poliedro

void stampaShortPathInfo(const Polyhedron& poly) {
    int edgeCount = 0;
    double totalDistance = 0.0;

    for (const auto& edge : poly.edges) {
        if (edge.ShortPath) {
            edgeCount++;
            const Vertex& v1 = poly.vertices[edge.origin];
            const Vertex& v2 = poly.vertices[edge.end];
            totalDistance += distanza(v1, v2);
        }
    }

    cout << "Numero di lati nello shortpath: " << edgeCount << endl;
    cout << "Distanza totale dello shortpath: " << totalDistance << endl;
}