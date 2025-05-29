#include "polyhedron_library.hpp"
#include "Utils.hpp"
#include "triangolazione.hpp"

#include <iostream>
#include <fstream>

using namespace std;
using namespace polyhedron_library;


int main()
{
    // user input
    vector<int> quadrupla;
    int numero;

    cout << "Inserisci 4 numeri interi :\n";
    for (int i = 0; i < 4; ++i) {
        cin >> numero;
        quadrupla.push_back(numero);
    }

    cout << "Hai inserito: ";
    for (int n : quadrupla) {
        cout << n << " ";
    }
    cout << endl;
	string risultato = controllaQuadrupla(quadrupla);
    if (risultato == "OK") {
        cout << "La quadrupla è valida.\n";
    } else {
        cout << "Errore: " << risultato << endl;
    }

    // create output files

    int q = quadrupla[1];

    Polyhedron solid;

    if(q == 3)
    {
        solid = tetraedro();
    }

    else if(q == 4)
    {
        solid = ottaedro();
    }

    else if(q == 5)
    {
        solid = icosaedro();
    }

    else
    {
        cerr << "unable to initialize solid" << endl;
        return 1;
    }

    Polyhedron geodetic = triangolazione(solid, 3);

    exportVertices(geodetic.vertices);
    exportEdges(geodetic.edges);
    exportFaces(geodetic.faces);
    exportPolyhedra(geodetic);

    
    
    
    return 0;

}