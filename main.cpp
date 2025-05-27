#include "polyhedron_library.hpp"
#include "Utils.hpp"

#include <iostream>

using namespace std;
using namespace polyhedron_library;


int main() {
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
}