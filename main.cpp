#include <iostream>

#include "polyhedron_library.hpp"
#include "Utils.hpp"
#include "triangolazione.hpp"

using namespace std;
using namespace polyhedron_library;

int main()
{
    // Input dell'utente

    vector<int> input;
    int numero;

    cout << "Inserisci 4 o 6 numeri interi: (se vuoi inserirne solo 4, premi qualsiasi carattere non numerico per inviare)\n";
    for (int i = 0; i < 6; ++i) {
        if (!(cin >> numero)) break;
        input.push_back(numero);
    }

    if (input.size() != 4 && input.size() != 6) {
        cerr << "Errore: inserire esattamente 4 o 6 numeri interi." << endl;
        return 1;
    }

    // Verifica quadrupla iniziale

    const vector<int> quadrupla = {input[0], input[1], input[2], input[3]};
    cout << "Hai inserito i parametri: ";
    for (int n : quadrupla) cout << n << " ";
    cout << endl;

    string risultato = controllaQuadrupla(quadrupla);
    if (risultato == "OK") {
        cout << "La quadrupla è valida.\n";
    } else {
        cout << "Errore: " << risultato << endl;
        return 1;
    }

    // Costruzione del solido platonico

    int q = quadrupla[1];
    int b = (quadrupla[2] > 0) ? quadrupla[2] : quadrupla[3];
    Polyhedron solid;
    if (q == 3) {
        solid = tetraedro();
    } else if (q == 4) {
        solid = ottaedro();
    } else if (q == 5) {
        solid = icosaedro();
    } else {
        cerr << "Errore: impossibile inizializzare il solido." << endl;
        return 1;
    }

    // Triangolazione del solido e normalizzazione dei vertici

    Polyhedron geodetic;
    if (quadrupla[2]==quadrupla[3]) {
        geodetic = triangolazione2(solid,b);
    }
    else {
        geodetic = triangolazione(solid, b);
    }

    for (Vertex& v : geodetic.vertices) {
        normalize(v);
    }

    // Esportazione delle informazioni sul poliedro geodetico

    string subfolderGeo = "Geodetic";
    exportVertices(geodetic.vertices, subfolderGeo);
    exportEdges(geodetic.edges, subfolderGeo);
    exportFaces(geodetic.faces, subfolderGeo);
    exportPolyhedra(geodetic, subfolderGeo);

    // Se l'input ha 6 valori, viene trovato il cammino più breve tra i due vertici specificati

    if (input.size() == 6)
    {
        int id_vertex1 = input[4];
        int id_vertex2 = input[5];
        verificaVertici(geodetic,id_vertex1,id_vertex2);
        shortestPath(geodetic,id_vertex1,id_vertex2);
        stampaShortPathInfo(geodetic);
        vector<int> vertexBool = getVertexShortPathFlags(geodetic);
        vector<int> edgeBool = getEdgeShortPathFlags(geodetic);
        exportParaviewFlags(geodetic, vertexBool, edgeBool, subfolderGeo);
    }
    else
    {
        exportParaview(geodetic, subfolderGeo);
    }
    
    // Se q == 3, viene creato ed esportato il corrispondente poliedro di Goldberg

    if (q == 3)
    {
        string subfolderGol = "Goldberg";
        Polyhedron goldberg = dualPolyhedron(geodetic);
        exportVertices(goldberg.vertices, subfolderGol);
        exportEdges(goldberg.edges, subfolderGol);
        exportFaces(goldberg.faces, subfolderGol);
        exportPolyhedra(goldberg, subfolderGol);
        exportParaview(goldberg, subfolderGol);
    }

    return 0;
}
