#include <gtest/gtest.h>
#include "polyhedron_library.hpp"
#include "Utils.hpp"
#include "triangolazione.hpp"

using namespace polyhedron_library;

TEST(TriangolazioneTest, SuddivisioneTetraedro) {
    Polyhedron p = tetraedro();

    int b = 2;

    // Applica la suddivisione
    Polyhedron p_triangolato = triangolazione(p, b);

    // Il tetraedro ha 4 facce, quindi ci aspettiamo 4 * 4 = 16 facce
    EXPECT_EQ(p_triangolato.faces.size(), 16);

    // Verifica che il numero di vertici sia aumentato
    EXPECT_EQ(p_triangolato.vertices.size(), 10);

    EXPECT_EQ(p_triangolato.edges.size(), 24); //tutta sta manfrina l'abbiamo fatta a mente potrebbe essere sbagliato :p
    int x = 3;
    Polyhedron p_triangolato2 = triangolazione(p, x);

    EXPECT_EQ(p_triangolato2.faces.size(), 36);

    EXPECT_EQ(p_triangolato2.vertices.size(), 20);

    EXPECT_EQ(p_triangolato2.edges.size(), 54);
}