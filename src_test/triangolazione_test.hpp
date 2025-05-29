#include <gtest/gtest.h>
#include "polyhedron_library.hpp"
#include "Utils.hpp"
#include "triangolazione.hpp"

using namespace polyhedron_library;

TEST(TriangolazioneTest, SuddivisioneTetraedro) {
    Polyhedron p = tetraedro();

    int b = 3;

    // Applica la suddivisione
    Polyhedron p_triangolato = suddividiTutteLeFacce(p, b);
    // Il tetraedro ha 4 facce, quindi ci aspettiamo 4 * 4 = 16 facce
    EXPECT_EQ(p_triangolato.faces.size(), 36);

    // Verifica che il numero di vertici sia aumentato
    EXPECT_EQ(p_triangolato.vertices.size(), 20);

    EXPECT_EQ(p_triangolato.edges.size(), 54); //tutta sta manfrina l'abbiamo fatta a mente potrebbe essere sbagliato :p
}