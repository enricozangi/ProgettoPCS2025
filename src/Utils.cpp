#include "Utils.hpp"
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

    bool terzoZero = (quadrupla[2] == 0);
    bool quartoZero = (quadrupla[3] == 0);

    if (terzoZero && quartoZero)
        return "Sia il terzo che il quarto numero sono zero";
    if (!terzoZero && !quartoZero)
        return "Nessuno zero tra terzo e quarto numero";

    return "OK";
}