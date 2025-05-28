#include <gtest/gtest.h>
#include "polyhedron_library.hpp"
#include "Utils.hpp"
#include "triangolazione.hpp"

using namespace polyhedron_library;

TEST(TriangolazioneTest, SuddivisioneTetraedro) {
    Polyhedron p = tetraedro();

    int b = 2;

    // Applica la suddivisione
    Polyhedron p_triangolato = suddividiTutteLeFacce(p, b);
    // Il tetraedro ha 4 facce, quindi ci aspettiamo 4 * 4 = 16 facce
    EXPECT_EQ(p_triangolato.faces.size(), 16);

    // Verifica che il numero di vertici sia aumentato
    // Numero atteso: (b + 1) * (b + 2) / 2 = 6 vertici per faccia
    // Ma i vertici possono essere condivisi tra le facce, quindi il numero totale
    // sarà inferiore a 4 * 6 = 24. Non possiamo determinare esattamente senza
    // conoscere l'implementazione, ma possiamo verificare che sia maggiore di quello iniziale
    EXPECT_EQ(p_triangolato.vertices.size(), 10);

    EXPECT_EQ(p_triangolato.edges.size(), 24); //tutta sta manfrina l'abbiamo fatta a mente potrebbe essere sbagliato :p
}